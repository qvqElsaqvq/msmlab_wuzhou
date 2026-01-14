// ***********************************************************************
// Assembly         : 
// Author           : Administrator
// Created          : 08-05-2021
//
// Last Modified By : Administrator
// Last Modified On : 08-05-2021
// ***********************************************************************
// <copyright file="Utility.h" company="Nokov">
//     Copyright (c) Nokov. All rights reserved.
// </copyright>
// <summary>辅助功能头文件，提供速度，加速度的外部计算支持</summary>
// ***********************************************************************

#pragma once

#include <list>
#include <fstream>
#include <string>
#include <cmath>

#define ERRORDATA	(9999999.999999) 
#define Mid(X) int(X/2)
#define FrameFactor 3  // 3,5,7
#define FPS 60

#define IN                                               //入参																			
#define OUT                                              //出参																						
#define IN_OUT                                           //入出参

struct Point
{
	double x;
	double y;
	double z;
	int	iFrame;
	std::string name;
};

struct Vel      //定义速度
{
	double Vx;
	double Vy;
	double Vz;
	double Vr;// |V|
	Vel() :Vx(0), Vy(0), Vz(0), Vr(0) {}

	void output(const Vel& record)
	{
		std::string v = "V(mm/s)";
		printf("%20s\tVx:%6.2F\tVy:%6.2F\tVz:%6.2F\tVr:%6.2F", v.c_str(), record.Vx, record.Vy, record.Vz, record.Vr);
		printf("\n");
	}

	friend std::ostream& operator << (std::ostream& os, const Vel& record)
	{
		os << "Vx:" << record.Vx << "\t";
		os << "Vy:" << record.Vy << "\t";
		os << "Vz:" << record.Vz << "\t";
		os << "Vr:" << record.Vr << "\t";
		os << std::endl;

		return os;
	}
};

struct Accel      //定义加速度
{
	double Ax;
	double Ay;
	double Az;
	double Ar; // |A|
	Accel() :Ax(0), Ay(0), Az(0), Ar(0) {}

	void output(const Accel& record)
	{
		std::string a = "A(mm/s^2)";
		printf("%20s\tAx:%6.2F\tAy:%6.2F\tAz:%6.2F\tAr:%6.2F\t", a.c_str(), record.Ax, record.Ay, record.Az, record.Ar);
		printf("\n");
	}

	friend std::ostream& operator << (std::ostream& os, const Accel& record)
	{
		os << "Ax:" << record.Ax << "\t";
		os << "Ay:" << record.Ay << "\t";
		os << "Az:" << record.Az << "\t";
		os << "Ar:" << record.Ar << "\t";
		os << std::endl;

		return os;
	}
};

// 封装的计算类，可派生自此基类实现自定义计算方式
template<class T>
class CalculateMethod
{
public:
	typedef T ValueType;
	typedef T& ReferenceType;

	CalculateMethod(double FR, int FF) :m_Points(nullptr), m_FPS(FR), m_FrameFactor(FF) {};

	int GetFrameFactor() const { return m_FrameFactor; };
	int GetFrameFPS() const { return m_FPS; };

	int tryToCalculate(IN Point* point, OUT ReferenceType variable)
	{
		if (nullptr == point)
			return 1;

		m_Points = point;

		if (m_FPS <= 0 || m_FPS >= 400)
			return 1;

		return calculate(variable);
	}

protected:
	virtual int calculate(OUT ReferenceType variable) = 0;

protected:
	Point* m_Points;
	double m_FPS;
	int m_FrameFactor;
};

class CalculateVelocity : public CalculateMethod<Vel>
{
public:
	CalculateVelocity(double FR, int FF) :CalculateMethod(FR, FF) {};

protected:
	virtual int calculate(OUT Vel& vel) override
	{
		int TR = m_FrameFactor / 2;
		int diff = fabs(m_Points[TR * 2].iFrame - m_Points[0].iFrame);
		if (diff == 0) {
			vel.Vx = 0;
			vel.Vy = 0;
			vel.Vz = 0;
			vel.Vr = 0;
			return 0;
		}
		vel.Vx = m_FPS * (m_Points[TR * 2].x - m_Points[0].x) / (diff);
		vel.Vy = m_FPS * (m_Points[TR * 2].y - m_Points[0].y) / (diff);
		vel.Vz = m_FPS * (m_Points[TR * 2].z - m_Points[0].z) / (diff);
		vel.Vr = sqrt(vel.Vx * vel.Vx + vel.Vy * vel.Vy + vel.Vz * vel.Vz);

		return 0;
	}
};

// 两帧计算法，从第二帧起实时计算
class CalculateVelocityByTwoFrame : public CalculateMethod<Vel>
{
public:
	CalculateVelocityByTwoFrame(double FR) :CalculateMethod(FR, 2) {};

protected:
	virtual int calculate(OUT Vel& vel) override
	{
		vel.Vx = m_FPS * (m_Points[1].x - m_Points[0].x);
		vel.Vy = m_FPS * (m_Points[1].y - m_Points[0].y);
		vel.Vz = m_FPS * (m_Points[1].z - m_Points[0].z);
		vel.Vr = sqrt(vel.Vx * vel.Vx + vel.Vy * vel.Vy + vel.Vz * vel.Vz);

		return 0;
	}
};

class CalculateAcceleration : public CalculateMethod<Accel>
{
public:
	CalculateAcceleration(double FR, int FF) :CalculateMethod(FR, FF) {};
protected:
	virtual int calculate(OUT Accel& accel) override
	{
		int TR = m_FrameFactor / 2;
		int diffFrame0 = fabs(m_Points[TR].iFrame - m_Points[0].iFrame);
		int diffFrame1 = fabs(m_Points[TR * 2].iFrame - m_Points[TR].iFrame);
		if (diffFrame0 == 0 || diffFrame1 == 0) {
			accel.Ax = 0;
			accel.Ay = 0;
			accel.Az = 0;
			accel.Ar = 0;
			return 0;
		}

		accel.Ax = ((m_Points[TR * 2].x - m_Points[TR].x) * m_FPS / diffFrame1 - (m_Points[TR].x - m_Points[0].x) * m_FPS / diffFrame0) * m_FPS / diffFrame1;
		accel.Ay = ((m_Points[TR * 2].y - m_Points[TR].y) * m_FPS / diffFrame1 - (m_Points[TR].y - m_Points[0].y) * m_FPS / diffFrame0) * m_FPS / diffFrame1;
		accel.Az = ((m_Points[TR * 2].z - m_Points[TR].z) * m_FPS / diffFrame1 - (m_Points[TR].z - m_Points[0].z) * m_FPS / diffFrame0) * m_FPS / diffFrame1;
		
		/*accel.Ax = m_FPS * m_FPS * (m_Points[TR * 2].x - 2 * m_Points[TR].x + m_Points[0].x) / (TR * TR);
		accel.Ay = m_FPS * m_FPS * (m_Points[TR * 2].y - 2 * m_Points[TR].y + m_Points[0].y) / (TR * TR);
		accel.Az = m_FPS * m_FPS * (m_Points[TR * 2].z - 2 * m_Points[TR].z + m_Points[0].z) / (TR * TR);*/
		accel.Ar = sqrt(accel.Ax * accel.Ax + accel.Ay * accel.Ay + accel.Az * accel.Az);
		return 0;
	}
};

// 滑动帧数组，存储待计算的数据，保留原始指针类型
class SlideFrameArray
{
public:

	explicit SlideFrameArray()
	{
		clear();
	}

	void clear()
	{
		_list.clear();
	}

	size_t cache(const Point& point)
	{
		_list.push_back(point);
		return _list.size();
	}

	size_t Cache(double x, double y, double z, int iFrame=0)
	{
		Point point = { 0 };
		point.x = x;
		point.y = y;
		point.z = z;
		point.iFrame = iFrame;

		return cache(point);
	}

	template<class T>
	bool tryToCalculate(OUT T& data, CalculateMethod<T>& method)
	{
		Point* inArray = nullptr;
		if (!frameArray(inArray, method.GetFrameFactor()))
		{
			if (inArray)
			{
				delete[] inArray;
				inArray = nullptr;
			}

			return false;
		}

		int retCount = method.tryToCalculate(inArray, data);
		if (retCount != 0)
		{
			delete[] inArray;
			return false;
		}

		delete[] inArray;
		return true;
	}

private:
	// the retArray need to be free by the user

	bool frameArray(OUT Point*& retArray, int frameFactor = FrameFactor)
	{
		if (_list.size() < frameFactor)
			return false;

		retArray = new Point[frameFactor];

		int index = 0;
		for (auto itor = _list.begin(); itor != _list.end() && index < frameFactor; ++itor, ++index)
		{
			retArray[index] = *itor;
		}

		// 向后滑动一格
		_list.pop_front();

		return true;
	}

private:
	std::list<Point> _list;
};






