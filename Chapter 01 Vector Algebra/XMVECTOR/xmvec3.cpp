/*
#include <windows.h> // for FLOAT definition
#include <DirectXMath.h>
#include <DirectXPackedVector.h>
#include <iostream>
using namespace std;

// Overload the  "<<" operators so that we can use cout to 
// output DirectX::XMVECTOR objects.
ostream& operator<<(ostream& os, DirectX::FXMVECTOR v)
{
	DirectX::XMFLOAT3 dest;
	XMStoreFloat3(&dest, v);

	os << "(" << dest.x << ", " << dest.y << ", " << dest.z << ")";
	return os;
}

int main()
{
	cout.setf(ios_base::boolalpha);

	// Check support for SSE2 (Pentium4, AMD K8, and above).
	if( !DirectX::XMVerifyCPUSupport() )
	{
		cout << "xna math not supported" << endl;
		return 0;
	}

	DirectX::XMVECTOR n = DirectX::XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f);
	DirectX::XMVECTOR u = DirectX::XMVectorSet(1.0f, 2.0f, 3.0f, 0.0f);
	DirectX::XMVECTOR v = DirectX::XMVectorSet(-2.0f, 1.0f, -3.0f, 0.0f);
	DirectX::XMVECTOR w = DirectX::XMVectorSet(0.707f, 0.707f, 0.0f, 0.0f);
	
	// Vector addition: DirectX::XMVECTOR operator + 
	DirectX::XMVECTOR a = u + v;

	// Vector subtraction: DirectX::XMVECTOR operator - 
	DirectX::XMVECTOR b = u - v;

	// Scalar multiplication: DirectX::XMVECTOR operator * 
	DirectX::XMVECTOR c = 10.0f*u;

	// ||u||
	DirectX::XMVECTOR L = DirectX::XMVector3Length(u);

	// d = u / ||u||
	DirectX::XMVECTOR d = DirectX::XMVector3Normalize(u);

	// s = u dot v
	DirectX::XMVECTOR s = DirectX::XMVector3Dot(u, v);

	// e = u x v
	DirectX::XMVECTOR e = DirectX::XMVector3Cross(u, v);

	// Find proj_n(w) and perp_n(w)
	DirectX::XMVECTOR projW;
	DirectX::XMVECTOR perpW;
	DirectX::XMVector3ComponentsFromNormal(&projW, &perpW, w, n);

	// Does projW + perpW == w?
	bool equal = DirectX::XMVector3Equal(projW + perpW, w) != 0;
	bool notEqual = DirectX::XMVector3NotEqual(projW + perpW, w) != 0;

	// The angle between projW and perpW should be 90 degrees.
	DirectX::XMVECTOR angleVec = DirectX::XMVector3AngleBetweenVectors(projW, perpW);
	float angleRadians = DirectX::XMVectorGetX(angleVec);
	float angleDegrees = XMConvertToDegrees(angleRadians);

	cout << "u                   = " << u << endl;
	cout << "v                   = " << v << endl;
	cout << "w                   = " << w << endl;
	cout << "n                   = " << n << endl;
	cout << "a = u + v           = " << a << endl;
	cout << "b = u - v           = " << b << endl;
	cout << "c = 10 * u          = " << c << endl;
	cout << "d = u / ||u||       = " << d << endl;
	cout << "e = u x v           = " << e << endl;
	cout << "L  = ||u||          = " << L << endl;
	cout << "s = u.v             = " << s << endl;
	cout << "projW               = " << projW << endl;
	cout << "perpW               = " << perpW << endl;
	cout << "projW + perpW == w  = " << equal << endl;
	cout << "projW + perpW != w  = " << notEqual << endl;
	cout << "angle               = " << angleDegrees << endl;

	return 0;
}*/