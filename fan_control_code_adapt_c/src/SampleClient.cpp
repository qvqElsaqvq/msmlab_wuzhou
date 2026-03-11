#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <cctype>

#include "nokov_bridge.h"
#include "DataStreamClient.h"

using ViconDataStreamSDK::CPP::Client;
using ViconDataStreamSDK::CPP::Result;
using ViconDataStreamSDK::CPP::Direction;

static constexpr int kOk = 0;
static constexpr int kErr = 1;

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static Client* g_client = nullptr;
static std::thread g_worker;
static std::atomic<bool> g_running{false};

static std::vector<std::string> g_subjects;
static std::unordered_map<std::string, std::string> g_subjectRootSegment;
static std::unordered_map<int, std::string> g_idToName;
static std::unordered_map<std::string, int> g_nameToId;

static std::unordered_map<int, RigidPose> g_latestPoseById;
static std::unordered_map<std::string, RigidPose> g_latestPoseByName;

static bool StrIContains(const std::string& s, const char* needle)
{
    if (!needle || needle[0] == '\0') return false;
    std::string a = s;
    std::string b = needle;
    std::transform(a.begin(), a.end(), a.begin(), [](unsigned char c){ return (char)std::tolower(c); });
    std::transform(b.begin(), b.end(), b.begin(), [](unsigned char c){ return (char)std::tolower(c); });
    return a.find(b) != std::string::npos;
}

static long long NowMs()
{
    using namespace std::chrono;
    return (long long)duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

static void RefreshSubjectsLocked()
{
    g_subjects.clear();
    g_subjectRootSegment.clear();
    g_idToName.clear();
    g_nameToId.clear();

    if (!g_client) return;

    auto outCount = g_client->GetSubjectCount();
    if (outCount.Result != Result::Success) return;

    std::vector<std::string> names;
    names.reserve(outCount.SubjectCount);

    for (unsigned int i = 0; i < outCount.SubjectCount; ++i)
    {
        auto outName = g_client->GetSubjectName(i);
        if (outName.Result != Result::Success) continue;
        names.push_back(outName.SubjectName);
    }

    std::sort(names.begin(), names.end());
    g_subjects = names;

    std::unordered_map<std::string, int> assigned;
    int nextId = 1;

    auto assignId = [&](const std::string& name, int forcedId) {
        if (assigned.count(name)) return;
        assigned[name] = forcedId;
        g_idToName[forcedId] = name;
        g_nameToId[name] = forcedId;
    };

    for (const auto& name : g_subjects)
    {
        if (name == "WUZHOUSHANG") assignId(name, 2);
    }
    for (const auto& name : g_subjects)
    {
        if (StrIContains(name, "dock")) assignId(name, 1);
    }

    for (const auto& name : g_subjects)
    {
        if (assigned.count(name)) continue;
        while (g_idToName.count(nextId)) ++nextId;
        assignId(name, nextId);
        ++nextId;
    }

    for (const auto& name : g_subjects)
    {
        auto outRoot = g_client->GetSubjectRootSegmentName(name);
        if (outRoot.Result != Result::Success) continue;
        g_subjectRootSegment[name] = outRoot.SegmentName;
    }
}

static void WorkerLoop()
{
    while (g_running.load())
    {
        Client* c = nullptr;
        pthread_mutex_lock(&g_mutex);
        c = g_client;
        pthread_mutex_unlock(&g_mutex);

        if (!c)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto outFrame = c->GetFrame();
        if (outFrame.Result != Result::Success)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        const long long ts = NowMs();
        int frameNumber = 0;
        auto outFrameNum = c->GetFrameNumber();
        if (outFrameNum.Result == Result::Success) frameNumber = (int)outFrameNum.FrameNumber;

        pthread_mutex_lock(&g_mutex);
        if (g_subjects.empty())
        {
            RefreshSubjectsLocked();
        }
        auto subjects = g_subjects;
        auto rootSegMap = g_subjectRootSegment;
        auto nameToId = g_nameToId;
        pthread_mutex_unlock(&g_mutex);

        std::unordered_map<int, RigidPose> posesById;
        std::unordered_map<std::string, RigidPose> posesByName;

        for (const auto& subject : subjects)
        {
            const auto itRoot = rootSegMap.find(subject);
            if (itRoot == rootSegMap.end()) continue;
            const auto itId = nameToId.find(subject);
            if (itId == nameToId.end()) continue;

            const std::string& segment = itRoot->second;

            auto outT = c->GetSegmentGlobalTranslation(subject, segment);
            auto outQ = c->GetSegmentGlobalRotationQuaternion(subject, segment);
            if (outT.Result != Result::Success || outQ.Result != Result::Success) continue;
            if (outT.Occluded || outQ.Occluded) continue;

            RigidPose p;
            p.id = itId->second;
            p.x = outT.Translation[0];
            p.y = outT.Translation[1];
            p.z = outT.Translation[2];
            p.qx = outQ.Rotation[0];
            p.qy = outQ.Rotation[1];
            p.qz = outQ.Rotation[2];
            p.qw = outQ.Rotation[3];
            p.timestamp = ts;
            p.frame = frameNumber;

            posesById[p.id] = p;
            posesByName[subject] = p;
            posesByName[segment] = p;
        }

        pthread_mutex_lock(&g_mutex);
        for (const auto& kv : posesById) g_latestPoseById[kv.first] = kv.second;
        for (const auto& kv : posesByName) g_latestPoseByName[kv.first] = kv.second;
        pthread_mutex_unlock(&g_mutex);
    }
}

int Nokov_Start(const char* server_ip)
{
    if (!server_ip || server_ip[0] == '\0')
    {
        printf("[VICON] server_ip empty\n");
        return kErr;
    }

    pthread_mutex_lock(&g_mutex);

    if (g_running.load())
    {
        pthread_mutex_unlock(&g_mutex);
        return kOk;
    }

    if (g_client)
    {
        g_client->Disconnect();
        delete g_client;
        g_client = nullptr;
    }

    g_client = new Client();
    g_subjects.clear();
    g_subjectRootSegment.clear();
    g_idToName.clear();
    g_nameToId.clear();
    g_latestPoseById.clear();
    g_latestPoseByName.clear();

    pthread_mutex_unlock(&g_mutex);

    auto outConn = g_client->Connect(server_ip);
    if (outConn.Result != Result::Success)
    {
        printf("[VICON] Connect failed. code=%d\n", (int)outConn.Result);
        pthread_mutex_lock(&g_mutex);
        delete g_client;
        g_client = nullptr;
        pthread_mutex_unlock(&g_mutex);
        return kErr;
    }

    g_client->EnableSegmentData();
    g_client->SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
    g_client->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);

    g_running.store(true);
    g_worker = std::thread(WorkerLoop);

    printf("[VICON] Connected to %s, receiving frames...\n", server_ip);
    return kOk;
}

void Nokov_Stop()
{
    pthread_mutex_lock(&g_mutex);
    if (!g_running.load())
    {
        pthread_mutex_unlock(&g_mutex);
        return;
    }
    g_running.store(false);
    if (g_client) g_client->Disconnect();
    pthread_mutex_unlock(&g_mutex);

    if (g_worker.joinable()) g_worker.join();

    pthread_mutex_lock(&g_mutex);
    if (g_client)
    {
        delete g_client;
        g_client = nullptr;
    }
    g_subjects.clear();
    g_subjectRootSegment.clear();
    g_idToName.clear();
    g_nameToId.clear();
    g_latestPoseById.clear();
    g_latestPoseByName.clear();
    pthread_mutex_unlock(&g_mutex);
}

bool Nokov_GetPoseById(int id, RigidPose& out)
{
    pthread_mutex_lock(&g_mutex);
    const auto it = g_latestPoseById.find(id);
    if (it == g_latestPoseById.end())
    {
        pthread_mutex_unlock(&g_mutex);
        return false;
    }
    out = it->second;
    pthread_mutex_unlock(&g_mutex);
    return true;
}

bool Nokov_GetPoseByName(const std::string& name, RigidPose& out)
{
    pthread_mutex_lock(&g_mutex);
    const auto it = g_latestPoseByName.find(name);
    if (it != g_latestPoseByName.end())
    {
        out = it->second;
        pthread_mutex_unlock(&g_mutex);
        return true;
    }
    pthread_mutex_unlock(&g_mutex);
    return false;
}
