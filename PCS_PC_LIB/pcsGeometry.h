#ifndef PCSGEOMETRY_H
#define PCSGEOMETRY_H

#include <math.h>

typedef float PointCoordinateType; //点云整合类型
typedef float ScalarType; //标量类型

template <class Type> class Tuple3Tpl
{
public:
	//坐标三元素元组
	union
	{
		struct
		{
			Type x, y, z;
		};
		Type u[3];
	};

	//! Default constructor
	/** Inits tuple to (0,0,0).
	**/
	inline Tuple3Tpl() : x(0), y(0), z(0) {}

	//! Constructor from a triplet of values
	/** Inits typle to (a,b,c).
	**/
	inline Tuple3Tpl(Type a, Type b, Type c) : x(a), y(b), z(c) {}

	//! Constructor from an array of 3 elements
	inline Tuple3Tpl(const Type p[]) : x(p[0]), y(p[1]), z(p[2]) {}

	//! Copy constructor
	inline Tuple3Tpl(const Tuple3Tpl& v) : x(v.x), y(v.y), z(v.z) {}
};

//3D矢量类模板
template <class Type> class Vector3Tpl : public Tuple3Tpl<Type>
{
public:
	//Don't ask me what other x, y, z or u members this class could
	//use but it seems necessary for compilation on some platforms...
	using Tuple3Tpl<Type>::x;
	using Tuple3Tpl<Type>::y;
	using Tuple3Tpl<Type>::z;
	using Tuple3Tpl<Type>::u;

	//起始矢量构造函数
	inline Vector3Tpl() : Tuple3Tpl<Type>() {}
	//坐标构造函数
	inline Vector3Tpl(Type x_, Type y_, Type z_) : Tuple3Tpl<Type>(x_, y_, z_) {}
	//三个元素数组的构造函数
	inline Vector3Tpl(const Type p[]) : Tuple3Tpl<Type>(p) {}
	//复制构造函数
	inline Vector3Tpl(const Vector3Tpl& v) : Tuple3Tpl<Type>(v) {}
	//int数组的构造函数 
	static inline Vector3Tpl Array(const int a[3]) { return Vector3Tpl(static_cast<Type>(a[0]), static_cast<Type>(a[1]), static_cast<Type>(a[2])); }
	//float数组的构造函数
	static inline Vector3Tpl Array(const float a[3]) { return Vector3Tpl(static_cast<Type>(a[0]), static_cast<Type>(a[1]), static_cast<Type>(a[2])); }
	//double数组的构造函数
	static inline Vector3Tpl Array(const double a[3]) { return Vector3Tpl(static_cast<Type>(a[0]), static_cast<Type>(a[1]), static_cast<Type>(a[2])); }
	//点积
	inline Type dot(const Vector3Tpl& v) const { return (x*v.x) + (y*v.y) + (z*v.z); }
	//X乘
	inline Vector3Tpl cross(const Vector3Tpl &v) const { return Vector3Tpl((y*v.z) - (z*v.y), (z*v.x) - (x*v.z), (x*v.y) - (y*v.x)); }
	//平方
	inline Type norm2() const { return (x*x) + (y*y) + (z*z); }
	//向量平方范数（强制双精度输出） （范数:所有向量元素绝对值中的最大值）
	inline double norm2d() const { return static_cast<double>(x)*static_cast<double>(x)+static_cast<double>(y)*static_cast<double>(y)+static_cast<double>(z)*static_cast<double>(z); }
	inline Type norm() const { return sqrt(norm2()); }
	//向量范数（强制双精度输出）
	inline double normd() const { return sqrt(norm2d()); }
	//统一化向量范数
	inline void normalize()
	{
		Type n = norm2();
		if (n>0)
			*this /= sqrt(n);
	}
	//归一化向量
	inline Vector3Tpl orthogonal() const
	{
		Vector3Tpl ort;
		vorthogonal(u, ort.u);
		return ort;
	}
	//逆运算
	inline Vector3Tpl operator - () const
	{
		Vector3Tpl V(-x, -y, -z);
		return V;
	}
	//加法运算
	inline Vector3Tpl& operator += (const Vector3Tpl& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	//减法运算符
	inline Vector3Tpl& operator -= (const Vector3Tpl& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}
	//乘法运算符（标量） 
	inline Vector3Tpl& operator *= (Type v)
	{
		x *= v;
		y *= v;
		z *= v;
		return *this;
	}
	//除法运算符（标量）
	inline Vector3Tpl& operator /= (Type v)
	{
		x /= v;
		y /= v;
		z /= v;
		return *this;
	}
	//加法算子（优先级高）
	inline Vector3Tpl operator + (const Vector3Tpl& v) const
	{
		return Vector3Tpl(x + v.x, y + v.y, z + v.z);
	}
	//减
	inline Vector3Tpl operator - (const Vector3Tpl& v) const
	{
		return Vector3Tpl(x - v.x, y - v.y, z - v.z);
	}
	//乘
	inline Vector3Tpl operator * (Type s) const
	{
		return Vector3Tpl(x*s, y*s, z*s);
	}
	//除
	inline Vector3Tpl operator / (Type s) const
	{
		return Vector3Tpl(x / s, y / s, z / s);
	}
	//X乘
	inline Vector3Tpl operator * (const Vector3Tpl& v) const
	{
		return cross(v);
	}
	//复制
	inline Vector3Tpl& operator = (const Vector3Tpl &v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}
	//点
	inline Type operator && (const Vector3Tpl &v) const
	{
		return dot(v);
	}
	//直接协调访问 
	inline Type& operator [] (unsigned i)
	{
		return u[i];
	}
	//直接坐标访问（常量） 
	inline const Type& operator [] (unsigned i) const
	{
		return u[i];
	}

	static inline void vdivide(const Type p[], Type s, Type r[])
	{
		r[0] = p[0] / s;
		r[1] = p[1] / s;
		r[2] = p[2] / s;
	}
	static inline void vdivide(Type p[], Type s)
	{
		p[0] /= s;
		p[1] /= s;
		p[2] /= s;
	}
	static inline void vmultiply(const Type p[], Type s, Type r[])
	{
		r[0] = p[0] * s;
		r[1] = p[1] * s;
		r[2] = p[2] * s;
	}
	static inline void vmultiply(Type p[], Type s)
	{
		p[0] *= s;
		p[1] *= s;
		p[2] *= s;
	}
	static inline Type vdot(const Type p[], const Type q[])
	{
		return (p[0] * q[0]) + (p[1] * q[1]) + (p[2] * q[2]);
	}
	static inline void vcross(const Type p[], const Type q[], Type r[])
	{
		r[0] = (p[1] * q[2]) - (p[2] * q[1]);
		r[1] = (p[2] * q[0]) - (p[0] * q[2]);
		r[2] = (p[0] * q[1]) - (p[1] * q[0]);
	}
	static inline void vcopy(const Type p[], Type q[])
	{
		q[0] = p[0];
		q[1] = p[1];
		q[2] = p[2];
	}
	static inline void vset(Type p[], Type s)
	{
		p[0] = p[1] = p[2] = s;
	}
	static inline void vset(Type p[], Type x, Type y, Type z)
	{
		p[0] = x;
		p[1] = y;
		p[2] = z;
	}
	static inline void vadd(const Type p[], const Type q[], Type r[])
	{
		r[0] = p[0] + q[0];
		r[1] = p[1] + q[1];
		r[2] = p[2] + q[2];
	}
	static inline void vsubstract(const Type p[], const Type q[], Type r[])
	{
		r[0] = p[0] - q[0];
		r[1] = p[1] - q[1];
		r[2] = p[2] - q[2];
	}
	static inline void vcombination(Type a, const Type p[], Type b, const Type q[], Type r[])
	{
		r[0] = (a*p[0]) + (b*q[0]);
		r[1] = (a*p[1]) + (b*q[1]);
		r[2] = (a*p[2]) + (b*q[2]);
	}
	static inline void vcombination(const Type p[], Type b, const Type q[], Type r[])
	{
		r[0] = p[0] + (b*q[0]);
		r[1] = p[1] + (b*q[1]);
		r[2] = p[2] + (b*q[2]);
	}
	static inline void vnormalize(Type p[])
	{
		Type n = vnorm2(p);
		if (n>0)
			vdivide(p, sqrt(n));
	}
	static inline Type vnorm2(const Type p[])
	{
		return (p[0] * p[0]) + (p[1] * p[1]) + (p[2] * p[2]);
	}
	static inline Type vdistance2(const Type p[], const Type q[])
	{
		return ((p[0] - q[0])*(p[0] - q[0])) + ((p[1] - q[1])*(p[1] - q[1])) + ((p[2] - q[2])*(p[2] - q[2]));
	}
	static inline Type vnorm(const Type p[])
	{
		return sqrt(vnorm2(p));
	}
	static inline Type vdistance(const Type p[], const Type q[])
	{
		return sqrt(vdistance2(p, q));
	}
	//矩形
	static inline void vorthogonal(const Type p[], Type q[])
	{
		if (fabs(p[0]) <= fabs(p[1]) && fabs(p[0]) <= fabs(p[2]))
		{
			q[0] = 0; q[1] = p[2]; q[2] = -p[1];
		}
		else if (fabs(p[1]) <= fabs(p[0]) && fabs(p[1]) <= fabs(p[2]))
		{
			q[0] = -p[2]; q[1] = 0; q[2] = p[0];
		}
		else
		{
			q[0] = p[1]; q[1] = -p[0]; q[2] = 0;
		}
		vnormalize(q);
	}

};

//标量算子对三维向量的乘法  
inline Vector3Tpl<float> operator * (float s, const Vector3Tpl<float> &v)
{
	return v*s;
}
inline Vector3Tpl<double> operator * (double s, const Vector3Tpl<double> &v)
{
	return v*s;
}

//2D矢量类模板
template <class Type> class Vector2Tpl
{
public:
	union
	{
		struct
		{
			Type x, y;
		};
		Type u[2];
	};

	inline Vector2Tpl(Type s = 0) : x(s), y(s) {}
	inline Vector2Tpl(Type x_, Type y_) : x(x_), y(y_) {}
	inline Vector2Tpl(const Vector2Tpl& v) : x(v.x), y(v.y) {}
	inline Type norm2() const
	{
		return (x*x) + (y*y);
	}
	inline Type norm() const
	{
		return sqrt(norm2());
	}
	inline void normalize()
	{
		Type n = norm2();
		if (n>0)
			*this /= sqrt(n);
	}
	inline Type dot(const Vector2Tpl& v) const
	{
		return (x*v.x) + (y*v.y);
	}
	inline Vector2Tpl& operator - ()
	{
		x = -x;
		y = -y;
		return *this;
	}
	inline Vector2Tpl& operator += (const Vector2Tpl& v)
	{
		x += v.x;
		y += v.y;
		return *this;
	}
	inline Vector2Tpl& operator -= (const Vector2Tpl& v)
	{
		x -= v.x;
		y -= v.y;
		return *this;
	}
	inline Vector2Tpl& operator *= (Type v)
	{
		x *= v;
		y *= v;
		return *this;
	}
	inline Vector2Tpl& operator /= (Type v)
	{
		x /= v;
		y /= v;
		return *this;
	}
	inline Vector2Tpl operator + (const Vector2Tpl& v) const
	{
		return Vector2Tpl(x + v.x, y + v.y);
	}
	inline Vector2Tpl operator - (const Vector2Tpl& v) const
	{
		return Vector2Tpl(x - v.x, y - v.y);
	}
	inline Vector2Tpl operator * (Type s) const
	{
		return Vector2Tpl(x*s, y*s);
	}
	inline Vector2Tpl operator / (Type s) const
	{
		return Vector2Tpl(x / s, y / s);
	}
	inline Vector2Tpl& operator = (const Vector2Tpl &v)
	{
		x = v.x;
		y = v.y;
		return *this;
	}
	inline Type& operator [] (unsigned i)
	{
		return u[i];
	}
	inline const Type& operator [] (unsigned i) const
	{
		return u[i];
	}
};


//默认3D矢量
typedef Vector3Tpl<PointCoordinateType> pcsVector3d;

//doule 3D矢量
typedef Vector3Tpl<double> pcsVector3D;

//默认2D矢量
typedef Vector2Tpl<PointCoordinateType> pcsVector2d;

//int 2D矢量
typedef Vector2Tpl<int> pcsVector2D;

#endif // PCSGEOMETRY_H
