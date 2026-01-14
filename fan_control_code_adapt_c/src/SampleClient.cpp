// src/SampleClient.cpp
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <unordered_map>
#include <vector>
#include <string>

#include "NokovSDKTypes.h"
#include "NokovSDKClient.h"
#include "Utility.h"

#include "nokov_bridge.h"

// ====== 回调声明 ======
static void DataHandler(sFrameOfMocapData* data, void* pUserData);

// ====== 全局 ======
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static NokovSDKClient* g_client = nullptr;

// ID -> Name
static std::unordered_map<int, std::string> g_rigidBodyIdMap;
// Name -> ID（方便按名字取）
static std::unordered_map<std::string, int> g_rigidBodyNameMap;

// 最新帧缓存：ID -> Pose
static std::unordered_map<int, RigidPose> g_latestPoseById;

static const char* SafeNameById(int id)
{
    auto it = g_rigidBodyIdMap.find(id);
    if (it == g_rigidBodyIdMap.end()) return "UNKNOWN";
    return it->second.c_str();
}

static void BuildRigidBodyMapFromDescriptions(sDataDescriptions* pData)
{
    if (!pData) return;

    g_rigidBodyIdMap.clear();
    g_rigidBodyNameMap.clear();

    for (int i = 0; i < pData->nDataDescriptions; ++i)
    {
        const auto& desc = pData->arrDataDescriptions[i];
        if (desc.type == Descriptor_RigidBody && desc.Data.RigidBodyDescription)
        {
            int id = desc.Data.RigidBodyDescription->ID;
            const char* name = desc.Data.RigidBodyDescription->szName ? desc.Data.RigidBodyDescription->szName : "UNKNOWN";
            g_rigidBodyIdMap[id] = name;
            g_rigidBodyNameMap[name] = id;
        }
    }

    printf("\n=== RigidBody Map (ID -> Name) ===\n");
    for (const auto& kv : g_rigidBodyIdMap)
        printf("  ID=%d  Name=%s\n", kv.first, kv.second.c_str());
    printf("=================================\n\n");
}

int Nokov_Start(const char* server_ip)
{
    if (!server_ip || server_ip[0] == '\0')
    {
        printf("[NOKOV] server_ip empty\n");
        return ErrorCode_Internal;
    }

    pthread_mutex_lock(&g_mutex);

    // release previous
    if (g_client)
    {
        g_client->Uninitialize();
        delete g_client;
        g_client = nullptr;
    }

    g_client = new NokovSDKClient();

    unsigned char ver[4] = {0};
    g_client->NokovSDKVersion(ver);
    printf("[NOKOV] SDK ver %d.%d.%d.%d\n", ver[0], ver[1], ver[2], ver[3]);

    int ret = g_client->Initialize((char*)server_ip);
    if (ret != ErrorCode_OK)
    {
        printf("[NOKOV] Initialize failed. code=%d\n", ret);
        pthread_mutex_unlock(&g_mutex);
        return ErrorCode_Internal;
    }

    sServerDescription sd;
    memset(&sd, 0, sizeof(sd));
    g_client->GetServerDescription(&sd);
    if (!sd.HostPresent)
    {
        printf("[NOKOV] Host not present\n");
        pthread_mutex_unlock(&g_mutex);
        return ErrorCode_Internal;
    }

    // descriptions -> build ID/Name map
    sDataDescriptions* pDesc = nullptr;
    g_client->GetDataDescriptions(&pDesc);
    if (pDesc)
    {
        BuildRigidBodyMapFromDescriptions(pDesc);
        g_client->FreeDataDescriptions(pDesc);
    }
    else
    {
        printf("[NOKOV] Warning: GetDataDescriptions returned null\n");
    }

    // register callback
    g_client->SetDataCallback(DataHandler, g_client);

    g_latestPoseById.clear();

    printf("[NOKOV] Connected to %s, receiving frames...\n", server_ip);

    pthread_mutex_unlock(&g_mutex);
    return ErrorCode_OK;
}

void Nokov_Stop()
{
    pthread_mutex_lock(&g_mutex);

    if (g_client)
    {
        g_client->Uninitialize();
        delete g_client;
        g_client = nullptr;
    }
    g_latestPoseById.clear();
    g_rigidBodyIdMap.clear();
    g_rigidBodyNameMap.clear();

    pthread_mutex_unlock(&g_mutex);
}

bool Nokov_GetPoseById(int id, RigidPose& out)
{
    pthread_mutex_lock(&g_mutex);
    auto it = g_latestPoseById.find(id);
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
    auto itName = g_rigidBodyNameMap.find(name);
    if (itName == g_rigidBodyNameMap.end())
    {
        pthread_mutex_unlock(&g_mutex);
        return false;
    }
    int id = itName->second;
    auto itPose = g_latestPoseById.find(id);
    if (itPose == g_latestPoseById.end())
    {
        pthread_mutex_unlock(&g_mutex);
        return false;
    }
    out = itPose->second;
    pthread_mutex_unlock(&g_mutex);
    return true;
}

// ====== SDK Callback ======
static void DataHandler(sFrameOfMocapData* data, void* /*pUserData*/)
{
    if (!data) return;

    pthread_mutex_lock(&g_mutex);

    for (int i = 0; i < data->nRigidBodies; ++i)
    {
        const sRigidBodyData& rb = data->RigidBodies[i];

        RigidPose p;
        p.id = rb.ID;
        p.x = rb.x; p.y = rb.y; p.z = rb.z;
        p.qx = rb.qx; p.qy = rb.qy; p.qz = rb.qz; p.qw = rb.qw;
        p.timestamp = data->iTimeStamp;
        p.frame = data->iFrame;

        g_latestPoseById[rb.ID] = p;
    }

    pthread_mutex_unlock(&g_mutex);
}
