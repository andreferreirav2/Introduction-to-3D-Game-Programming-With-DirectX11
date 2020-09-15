//-------------------------------------------------------------------------------------
// XNACollision.cpp
//  
// An opimtized collision library based on XNAMath
//  
// Microsoft XNA Developer Connection
// Copyright (c) Microsoft Corporation. All rights reserved.
//-------------------------------------------------------------------------------------

//#include "DXUT.h"
#include <Windows.h>
#include <cfloat>
#include "xnacollision.h"

namespace XNA
{

static const DirectX::XMVECTOR g_UnitQuaternionEpsilon =
{
    1.0e-4f, 1.0e-4f, 1.0e-4f, 1.0e-4f
};
static const DirectX::XMVECTOR g_UnitVectorEpsilon =
{
    1.0e-4f, 1.0e-4f, 1.0e-4f, 1.0e-4f
};
static const DirectX::XMVECTOR g_UnitPlaneEpsilon =
{
    1.0e-4f, 1.0e-4f, 1.0e-4f, 1.0e-4f
};


//-----------------------------------------------------------------------------
// Return TRUE if any of the elements of a 3 vector are equal to 0xffffffff.
// Slightly more efficient than using DirectX::XMVector3EqualInt.
//-----------------------------------------------------------------------------
static inline BOOL DirectX::XMVector3AnyTrue( FXMVECTOR V )
{
    DirectX::XMVECTOR C;

    // Duplicate the fourth element from the first element.
    C = DirectX::XMVectorSwizzle( V, 0, 1, 2, 0 );

    return XMComparisonAnyTrue( DirectX::XMVector4EqualIntR( C, DirectX::XMVectorTrueInt() ) );
}



//-----------------------------------------------------------------------------
// Return TRUE if all of the elements of a 3 vector are equal to 0xffffffff.
// Slightly more efficient than using DirectX::XMVector3EqualInt.
//-----------------------------------------------------------------------------
static inline BOOL DirectX::XMVector3AllTrue( FXMVECTOR V )
{
    DirectX::XMVECTOR C;

    // Duplicate the fourth element from the first element.
    C = DirectX::XMVectorSwizzle( V, 0, 1, 2, 0 );

    return XMComparisonAllTrue( DirectX::XMVector4EqualIntR( C, DirectX::XMVectorTrueInt() ) );
}



//-----------------------------------------------------------------------------
// Return TRUE if the vector is a unit vector (length == 1).
//-----------------------------------------------------------------------------
static inline BOOL DirectX::XMVector3IsUnit( FXMVECTOR V )
{
    DirectX::XMVECTOR Difference = DirectX::XMVector3Length( V ) - DirectX::XMVectorSplatOne();

    return DirectX::XMVector4Less( DirectX::XMVectorAbs( Difference ), g_UnitVectorEpsilon );
}



//-----------------------------------------------------------------------------
// Return TRUE if the quaterion is a unit quaternion.
//-----------------------------------------------------------------------------
static inline BOOL XMQuaternionIsUnit( FXMVECTOR Q )
{
    DirectX::XMVECTOR Difference = DirectX::XMVector4Length( Q ) - DirectX::XMVectorSplatOne();

    return DirectX::XMVector4Less( DirectX::XMVectorAbs( Difference ), g_UnitQuaternionEpsilon );
}



//-----------------------------------------------------------------------------
// Return TRUE if the plane is a unit plane.
//-----------------------------------------------------------------------------
static inline BOOL XMPlaneIsUnit( FXMVECTOR Plane )
{
    DirectX::XMVECTOR Difference = DirectX::XMVector3Length( Plane ) - DirectX::XMVectorSplatOne();

    return DirectX::XMVector4Less( DirectX::XMVectorAbs( Difference ), g_UnitPlaneEpsilon );
}



//-----------------------------------------------------------------------------
// Transform a plane by a rotation and translation.
//-----------------------------------------------------------------------------
static inline DirectX::XMVECTOR TransformPlane( FXMVECTOR Plane, DirectX::FXMVECTOR Rotation, DirectX::FXMVECTOR Translation )
{
    DirectX::XMVECTOR Normal = DirectX::XMVector3Rotate( Plane, Rotation );
    DirectX::XMVECTOR D = DirectX::XMVectorSplatW( Plane ) - DirectX::XMVector3Dot( Normal, Translation );

    return DirectX::XMVectorInsert( Normal, D, 0, 0, 0, 0, 1 );
}



//-----------------------------------------------------------------------------
// Return the point on the line segement (S1, S2) nearest the point P.
//-----------------------------------------------------------------------------
static inline DirectX::XMVECTOR PointOnLineSegmentNearestPoint( FXMVECTOR S1, DirectX::FXMVECTOR S2, DirectX::FXMVECTOR P )
{
    DirectX::XMVECTOR Dir = S2 - S1;
    DirectX::XMVECTOR Projection = ( DirectX::XMVector3Dot( P, Dir ) - DirectX::XMVector3Dot( S1, Dir ) );
    DirectX::XMVECTOR LengthSq = DirectX::XMVector3Dot( Dir, Dir );

    DirectX::XMVECTOR t = Projection * DirectX::XMVectorReciprocal( LengthSq );
    DirectX::XMVECTOR Point = S1 + t * Dir;

    // t < 0
    DirectX::XMVECTOR SelectS1 = DirectX::XMVectorLess( Projection, DirectX::XMVectorZero() );
    Point = DirectX::XMVectorSelect( Point, S1, SelectS1 );

    // t > 1
    DirectX::XMVECTOR SelectS2 = DirectX::XMVectorGreater( Projection, LengthSq );
    Point = DirectX::XMVectorSelect( Point, S2, SelectS2 );

    return Point;
}



//-----------------------------------------------------------------------------
// Test if the point (P) on the plane of the triangle is inside the triangle 
// (V0, V1, V2).
//-----------------------------------------------------------------------------
static inline DirectX::XMVECTOR PointOnPlaneInsideTriangle( FXMVECTOR P, DirectX::FXMVECTOR V0, DirectX::FXMVECTOR V1, CXMVECTOR V2 )
{
    // Compute the triangle normal.
    DirectX::XMVECTOR N = DirectX::XMVector3Cross( V2 - V0, V1 - V0 );

    // Compute the cross products of the vector from the base of each edge to 
    // the point with each edge vector.
    DirectX::XMVECTOR C0 = DirectX::XMVector3Cross( P - V0, V1 - V0 );
    DirectX::XMVECTOR C1 = DirectX::XMVector3Cross( P - V1, V2 - V1 );
    DirectX::XMVECTOR C2 = DirectX::XMVector3Cross( P - V2, V0 - V2 );

    // If the cross product points in the same direction as the normal the the
    // point is inside the edge (it is zero if is on the edge).
    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();
    DirectX::XMVECTOR Inside0 = DirectX::XMVectorGreaterOrEqual( DirectX::XMVector3Dot( C0, N ), Zero );
    DirectX::XMVECTOR Inside1 = DirectX::XMVectorGreaterOrEqual( DirectX::XMVector3Dot( C1, N ), Zero );
    DirectX::XMVECTOR Inside2 = DirectX::XMVectorGreaterOrEqual( DirectX::XMVector3Dot( C2, N ), Zero );

    // If the point inside all of the edges it is inside.
    return DirectX::XMVectorAndInt( DirectX::XMVectorAndInt( Inside0, Inside1 ), Inside2 );
}



//-----------------------------------------------------------------------------
// Find the approximate smallest enclosing bounding sphere for a set of 
// points. Exact computation of the smallest enclosing bounding sphere is 
// possible but is slower and requires a more complex algorithm.
// The algorithm is based on  Jack Ritter, "An Efficient Bounding Sphere", 
// Graphics Gems.
//-----------------------------------------------------------------------------
VOID ComputeBoundingSphereFromPoints( Sphere* pOut, UINT Count, const DirectX::XMFLOAT3* pPoints, UINT Stride )
{
    XMASSERT( pOut );
    XMASSERT( Count > 0 );
    XMASSERT( pPoints );

    // Find the points with minimum and maximum x, y, and z
    DirectX::XMVECTOR MinX, MaxX, MinY, MaxY, MinZ, MaxZ;

    MinX = MaxX = MinY = MaxY = MinZ = MaxZ = XMLoadFloat3( pPoints );

    for( UINT i = 1; i < Count; i++ )
    {
        DirectX::XMVECTOR Point = XMLoadFloat3( ( DirectX::XMFLOAT3* )( ( BYTE* )pPoints + i * Stride ) );

        float px = DirectX::XMVectorGetX( Point );
        float py = DirectX::XMVectorGetY( Point );
        float pz = DirectX::XMVectorGetZ( Point );

        if( px < DirectX::XMVectorGetX( MinX ) )
            MinX = Point;

        if( px > DirectX::XMVectorGetX( MaxX ) )
            MaxX = Point;

        if( py < DirectX::XMVectorGetY( MinY ) )
            MinY = Point;

        if( py > DirectX::XMVectorGetY( MaxY ) )
            MaxY = Point;

        if( pz < DirectX::XMVectorGetZ( MinZ ) )
            MinZ = Point;

        if( pz > DirectX::XMVectorGetZ( MaxZ ) )
            MaxZ = Point;
    }

    // Use the min/max pair that are farthest apart to form the initial sphere.
    DirectX::XMVECTOR DeltaX = MaxX - MinX;
    DirectX::XMVECTOR DistX = DirectX::XMVector3Length( DeltaX );

    DirectX::XMVECTOR DeltaY = MaxY - MinY;
    DirectX::XMVECTOR DistY = DirectX::XMVector3Length( DeltaY );

    DirectX::XMVECTOR DeltaZ = MaxZ - MinZ;
    DirectX::XMVECTOR DistZ = DirectX::XMVector3Length( DeltaZ );

    DirectX::XMVECTOR Center;
    DirectX::XMVECTOR Radius;

    if( DirectX::XMVector3Greater( DistX, DistY ) )
    {
        if( DirectX::XMVector3Greater( DistX, DistZ ) )
        {
            // Use min/max x.
            Center = ( MaxX + MinX ) * 0.5f;
            Radius = DistX * 0.5f;
        }
        else
        {
            // Use min/max z.
            Center = ( MaxZ + MinZ ) * 0.5f;
            Radius = DistZ * 0.5f;
        }
    }
    else // Y >= X
    {
        if( DirectX::XMVector3Greater( DistY, DistZ ) )
        {
            // Use min/max y.
            Center = ( MaxY + MinY ) * 0.5f;
            Radius = DistY * 0.5f;
        }
        else
        {
            // Use min/max z.
            Center = ( MaxZ + MinZ ) * 0.5f;
            Radius = DistZ * 0.5f;
        }
    }

    // Add any points not inside the sphere.
    for( UINT i = 0; i < Count; i++ )
    {
        DirectX::XMVECTOR Point = XMLoadFloat3( ( DirectX::XMFLOAT3* )( ( BYTE* )pPoints + i * Stride ) );

        DirectX::XMVECTOR Delta = Point - Center;

        DirectX::XMVECTOR Dist = DirectX::XMVector3Length( Delta );

        if( DirectX::XMVector3Greater( Dist, Radius ) )
        {
            // Adjust sphere to include the new point.
            Radius = ( Radius + Dist ) * 0.5f;
            Center += ( DirectX::XMVectorReplicate( 1.0f ) - Radius * DirectX::XMVectorReciprocal( Dist ) ) * Delta;
        }
    }

    XMStoreFloat3( &pOut->Center, Center );
    XMStoreFloat( &pOut->Radius, Radius );

    return;
}



//-----------------------------------------------------------------------------
// Find the minimum axis aligned bounding box containing a set of points.
//-----------------------------------------------------------------------------
VOID ComputeBoundingAxisAlignedBoxFromPoints( AxisAlignedBox* pOut, UINT Count, const DirectX::XMFLOAT3* pPoints, UINT Stride )
{
    XMASSERT( pOut );
    XMASSERT( Count > 0 );
    XMASSERT( pPoints );

    // Find the minimum and maximum x, y, and z
    DirectX::XMVECTOR vMin, vMax;

    vMin = vMax = XMLoadFloat3( pPoints );

    for( UINT i = 1; i < Count; i++ )
    {
        DirectX::XMVECTOR Point = XMLoadFloat3( ( DirectX::XMFLOAT3* )( ( BYTE* )pPoints + i * Stride ) );

        vMin = DirectX::XMVectorMin( vMin, Point );
        vMax = DirectX::XMVectorMax( vMax, Point );
    }

    // Store center and extents.
    XMStoreFloat3( &pOut->Center, ( vMin + vMax ) * 0.5f );
    XMStoreFloat3( &pOut->Extents, ( vMax - vMin ) * 0.5f );

    return;
}



//-----------------------------------------------------------------------------
static inline BOOL SolveCubic( FLOAT e, FLOAT f, FLOAT g, FLOAT* t, FLOAT* u, FLOAT* v )
{
    FLOAT p, q, h, rc, d, theta, costh3, sinth3;

    p = f - e * e / 3.0f;
    q = g - e * f / 3.0f + e * e * e * 2.0f / 27.0f;
    h = q * q / 4.0f + p * p * p / 27.0f;

    if( h > 0.0 )
    {
        return FALSE; // only one real root
    }

    if( ( h == 0.0 ) && ( q == 0.0 ) ) // all the same root
    {
        *t = - e / 3;
        *u = - e / 3;
        *v = - e / 3;

        return TRUE;
    }

    d = sqrtf( q * q / 4.0f - h );
    if( d < 0 )
        rc = -powf( -d, 1.0f / 3.0f );
    else
        rc = powf( d, 1.0f / 3.0f );

    theta = acosf( -q / ( 2.0f * d ) );
    costh3 = cosf( theta / 3.0f );
    sinth3 = sqrtf( 3.0f ) * sinf( theta / 3.0f );
    *t = 2.0f * rc * costh3 - e / 3.0f;
    *u = -rc * ( costh3 + sinth3 ) - e / 3.0f;
    *v = -rc * ( costh3 - sinth3 ) - e / 3.0f;

    return TRUE;
}



//-----------------------------------------------------------------------------
static inline DirectX::XMVECTOR CalculateEigenVector( FLOAT m11, FLOAT m12, FLOAT m13,
                                             FLOAT m22, FLOAT m23, FLOAT m33, FLOAT e )
{
    FLOAT f1, f2, f3;

    FLOAT fTmp[3];
    fTmp[0] = ( FLOAT )( m12 * m23 - m13 * ( m22 - e ) );
    fTmp[1] = ( FLOAT )( m13 * m12 - m23 * ( m11 - e ) );
    fTmp[2] = ( FLOAT )( ( m11 - e ) * ( m22 - e ) - m12 * m12 );

    DirectX::XMVECTOR vTmp = XMLoadFloat3( (DirectX::XMFLOAT3*)fTmp );

    if( DirectX::XMVector3Equal( vTmp, DirectX::XMVectorZero() ) ) // planar or linear
    {
        // we only have one equation - find a valid one
        if( ( m11 - e != 0.0 ) || ( m12 != 0.0 ) || ( m13 != 0.0 ) )
        {
            f1 = m11 - e; f2 = m12; f3 = m13;
        }
        else if( ( m12 != 0.0 ) || ( m22 - e != 0.0 ) || ( m23 != 0.0 ) )
        {
            f1 = m12; f2 = m22 - e; f3 = m23;
        }
        else if( ( m13 != 0.0 ) || ( m23 != 0.0 ) || ( m33 - e != 0.0 ) )
        {
            f1 = m13; f2 = m23; f3 = m33 - e;
        }
        else
        {
            // error, we'll just make something up - we have NO context
            f1 = 1.0; f2 = 0.0; f3 = 0.0;
        }

        if( f1 == 0.0 )
            vTmp = DirectX::XMVectorSetX( vTmp, 0.0f );
        else
            vTmp = DirectX::XMVectorSetX( vTmp, 1.0f );

        if( f2 == 0.0 )
            vTmp = DirectX::XMVectorSetY( vTmp, 0.0f );
        else
            vTmp = DirectX::XMVectorSetY( vTmp, 1.0f );

        if( f3 == 0.0 )
        {
            vTmp = DirectX::XMVectorSetZ( vTmp, 0.0f );
            // recalculate y to make equation work
            if( m12 != 0.0 )
                vTmp = DirectX::XMVectorSetY( vTmp, ( FLOAT )( -f1 / f2 ) );
        }
        else
        {
            vTmp = DirectX::XMVectorSetZ( vTmp, ( FLOAT )( ( f2 - f1 ) / f3 ) );
        }
    }

    if( DirectX::XMVectorGetX( DirectX::XMVector3LengthSq( vTmp ) ) > 1e-5f )
    {
        return DirectX::XMVector3Normalize( vTmp );
    }
    else
    {
        // Multiply by a value large enough to make the vector non-zero.
        vTmp *= 1e5f;
        return DirectX::XMVector3Normalize( vTmp );
    }
}



//-----------------------------------------------------------------------------
static inline BOOL CalculateEigenVectors( FLOAT m11, FLOAT m12, FLOAT m13,
                                          FLOAT m22, FLOAT m23, FLOAT m33,
                                          FLOAT e1, FLOAT e2, FLOAT e3,
                                          DirectX::XMVECTOR* pV1, DirectX::XMVECTOR* pV2, DirectX::XMVECTOR* pV3 )
{
    DirectX::XMVECTOR vTmp, vUp, vRight;

    BOOL v1z, v2z, v3z, e12, e13, e23;

    vUp = DirectX::XMVectorSetBinaryConstant( 0, 1, 0, 0 );
    vRight = DirectX::XMVectorSetBinaryConstant( 1, 0, 0, 0 );

    *pV1 = CalculateEigenVector( m11, m12, m13, m22, m23, m33, e1 );
    *pV2 = CalculateEigenVector( m11, m12, m13, m22, m23, m33, e2 );
    *pV3 = CalculateEigenVector( m11, m12, m13, m22, m23, m33, e3 );

    v1z = v2z = v3z = FALSE;

    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();

    if ( DirectX::XMVector3Equal( *pV1, Zero ) )
        v1z = TRUE;

    if ( DirectX::XMVector3Equal( *pV2, Zero ) )
        v2z = TRUE;

    if ( DirectX::XMVector3Equal( *pV3, Zero ))
        v3z = TRUE;

    e12 = ( fabsf( DirectX::XMVectorGetX( DirectX::XMVector3Dot( *pV1, *pV2 ) ) ) > 0.1f ); // check for non-orthogonal vectors
    e13 = ( fabsf( DirectX::XMVectorGetX( DirectX::XMVector3Dot( *pV1, *pV3 ) ) ) > 0.1f );
    e23 = ( fabsf( DirectX::XMVectorGetX( DirectX::XMVector3Dot( *pV2, *pV3 ) ) ) > 0.1f );

    if( ( v1z && v2z && v3z ) || ( e12 && e13 && e23 ) ||
        ( e12 && v3z ) || ( e13 && v2z ) || ( e23 && v1z ) ) // all eigenvectors are 0- any basis set
    {
        *pV1 = DirectX::XMVectorSetBinaryConstant( 1, 0, 0, 0 );
        *pV2 = DirectX::XMVectorSetBinaryConstant( 0, 1, 0, 0 );
        *pV3 = DirectX::XMVectorSetBinaryConstant( 0, 0, 1, 0 );
        return TRUE;
    }

    if( v1z && v2z )
    {
        vTmp = DirectX::XMVector3Cross( vUp, *pV3 );
        if( DirectX::XMVectorGetX( DirectX::XMVector3LengthSq( vTmp ) ) < 1e-5f )
        {
            vTmp = DirectX::XMVector3Cross( vRight, *pV3 );
        }
        *pV1 = DirectX::XMVector3Normalize( vTmp );
        *pV2 = DirectX::XMVector3Cross( *pV3, *pV1 );
        return TRUE;
    }

    if( v3z && v1z )
    {
        vTmp = DirectX::XMVector3Cross( vUp, *pV2 );
        if( DirectX::XMVectorGetX( DirectX::XMVector3LengthSq( vTmp ) ) < 1e-5f )
        {
            vTmp = DirectX::XMVector3Cross( vRight, *pV2 );
        }
        *pV3 = DirectX::XMVector3Normalize( vTmp );
        *pV1 = DirectX::XMVector3Cross( *pV2, *pV3 );
        return TRUE;
    }

    if( v2z && v3z )
    {
        vTmp = DirectX::XMVector3Cross( vUp, *pV1 );
        if( DirectX::XMVectorGetX( DirectX::XMVector3LengthSq( vTmp ) ) < 1e-5f )
        {
            vTmp = DirectX::XMVector3Cross( vRight, *pV1 );
        }
        *pV2 = DirectX::XMVector3Normalize( vTmp );
        *pV3 = DirectX::XMVector3Cross( *pV1, *pV2 );
        return TRUE;
    }

    if( ( v1z ) || e12 )
    {
        *pV1 = DirectX::XMVector3Cross( *pV2, *pV3 );
        return TRUE;
    }

    if( ( v2z ) || e23 )
    {
        *pV2 = DirectX::XMVector3Cross( *pV3, *pV1 );
        return TRUE;
    }

    if( ( v3z ) || e13 )
    {
        *pV3 = DirectX::XMVector3Cross( *pV1, *pV2 );
        return TRUE;
    }

    return TRUE;
}



//-----------------------------------------------------------------------------
static inline BOOL CalculateEigenVectorsFromCovarianceMatrix( FLOAT Cxx, FLOAT Cyy, FLOAT Czz,
                                                              FLOAT Cxy, FLOAT Cxz, FLOAT Cyz,
                                                              DirectX::XMVECTOR* pV1, DirectX::XMVECTOR* pV2, DirectX::XMVECTOR* pV3 )
{
    FLOAT e, f, g, ev1, ev2, ev3;

    // Calculate the eigenvalues by solving a cubic equation.
    e = -( Cxx + Cyy + Czz );
    f = Cxx * Cyy + Cyy * Czz + Czz * Cxx - Cxy * Cxy - Cxz * Cxz - Cyz * Cyz;
    g = Cxy * Cxy * Czz + Cxz * Cxz * Cyy + Cyz * Cyz * Cxx - Cxy * Cyz * Cxz * 2.0f - Cxx * Cyy * Czz;

    if( !SolveCubic( e, f, g, &ev1, &ev2, &ev3 ) )
    {
        // set them to arbitrary orthonormal basis set
        *pV1 = DirectX::XMVectorSetBinaryConstant( 1, 0, 0, 0 );
        *pV2 = DirectX::XMVectorSetBinaryConstant( 0, 1, 0, 0 );
        *pV3 = DirectX::XMVectorSetBinaryConstant( 0, 0, 1, 0 );
        return FALSE;
    }

    return CalculateEigenVectors( Cxx, Cxy, Cxz, Cyy, Cyz, Czz, ev1, ev2, ev3, pV1, pV2, pV3 );
}



//-----------------------------------------------------------------------------
// Find the approximate minimum oriented bounding box containing a set of 
// points.  Exact computation of minimum oriented bounding box is possible but 
// is slower and requires a more complex algorithm.
// The algorithm works by computing the inertia tensor of the points and then
// using the eigenvectors of the intertia tensor as the axes of the box.
// Computing the intertia tensor of the convex hull of the points will usually 
// result in better bounding box but the computation is more complex. 
// Exact computation of the minimum oriented bounding box is possible but the
// best know algorithm is O(N^3) and is significanly more complex to implement.
//-----------------------------------------------------------------------------
VOID ComputeBoundingOrientedBoxFromPoints( OrientedBox* pOut, UINT Count, const DirectX::XMFLOAT3* pPoints, UINT Stride )
{
    static CONST XMVECTORI32 PermuteXXY =
                 {
                    DirectX::XM_PERMUTE_0X, DirectX::XM_PERMUTE_0X, DirectX::XM_PERMUTE_0Y, DirectX::XM_PERMUTE_0W
                 };
    static CONST XMVECTORI32 PermuteYZZ =
                 {
                    DirectX::XM_PERMUTE_0Y, DirectX::XM_PERMUTE_0Z, DirectX::XM_PERMUTE_0Z, DirectX::XM_PERMUTE_0W
                 };

    XMASSERT( pOut );
    XMASSERT( Count > 0 );
    XMASSERT( pPoints );

    DirectX::XMVECTOR CenterOfMass = DirectX::XMVectorZero();

    // Compute the center of mass and inertia tensor of the points.
    for( UINT i = 0; i < Count; i++ )
    {
        DirectX::XMVECTOR Point = XMLoadFloat3( ( DirectX::XMFLOAT3* )( ( BYTE* )pPoints + i * Stride ) );

        CenterOfMass += Point;
    }

    CenterOfMass *= DirectX::XMVectorReciprocal( DirectX::XMVectorReplicate( FLOAT( Count ) ) );

    // Compute the inertia tensor of the points around the center of mass.
    // Using the center of mass is not strictly necessary, but will hopefully
    // improve the stability of finding the eigenvectors.
    DirectX::XMVECTOR XX_YY_ZZ = DirectX::XMVectorZero();
    DirectX::XMVECTOR XY_XZ_YZ = DirectX::XMVectorZero();

    for( UINT i = 0; i < Count; i++ )
    {
        DirectX::XMVECTOR Point = XMLoadFloat3( ( DirectX::XMFLOAT3* )( ( BYTE* )pPoints + i * Stride ) ) - CenterOfMass;

        XX_YY_ZZ += Point * Point;

        DirectX::XMVECTOR XXY = DirectX::XMVectorPermute( Point, Point, PermuteXXY );
        DirectX::XMVECTOR YZZ = DirectX::XMVectorPermute( Point, Point, PermuteYZZ );

        XY_XZ_YZ += XXY * YZZ;
    }

    DirectX::XMVECTOR v1, v2, v3;

    // Compute the eigenvectors of the inertia tensor.
    CalculateEigenVectorsFromCovarianceMatrix( DirectX::XMVectorGetX( XX_YY_ZZ ), DirectX::XMVectorGetY( XX_YY_ZZ ),
                                               DirectX::XMVectorGetZ( XX_YY_ZZ ),
                                               DirectX::XMVectorGetX( XY_XZ_YZ ), DirectX::XMVectorGetY( XY_XZ_YZ ),
                                               DirectX::XMVectorGetZ( XY_XZ_YZ ),
                                               &v1, &v2, &v3 );

    // Put them in a matrix.
    DirectX::XMMATRIX R;

    R.r[0] = DirectX::XMVectorSetW( v1, 0.f );
    R.r[1] = DirectX::XMVectorSetW( v2, 0.f );
    R.r[2] = DirectX::XMVectorSetW( v3, 0.f );
    R.r[3] = DirectX::XMVectorSetBinaryConstant( 0, 0, 0, 1 );

    // Multiply by -1 to convert the matrix into a right handed coordinate 
    // system (Det ~= 1) in case the eigenvectors form a left handed 
    // coordinate system (Det ~= -1) because XMQuaternionRotationMatrix only 
    // works on right handed matrices.
    DirectX::XMVECTOR Det = DirectX::XMMatrixDeterminant( R );

    if( DirectX::XMVector4Less( Det, DirectX::XMVectorZero() ) )
    {
        const XMVECTORF32 VectorNegativeOne =
        {
            -1.0f, -1.0f, -1.0f, -1.0f
        };

        R.r[0] *= VectorNegativeOne;
        R.r[1] *= VectorNegativeOne;
        R.r[2] *= VectorNegativeOne;
    }

    // Get the rotation quaternion from the matrix.
    DirectX::XMVECTOR Orientation = XMQuaternionRotationMatrix( R );

    // Make sure it is normal (in case the vectors are slightly non-orthogonal).
    Orientation = XMQuaternionNormalize( Orientation );

    // Rebuild the rotation matrix from the quaternion.
    R = DirectX::XMMatrixRotationQuaternion( Orientation );

    // Build the rotation into the rotated space.
    DirectX::XMMATRIX InverseR = DirectX::XMMatrixTranspose( R );

    // Find the minimum OBB using the eigenvectors as the axes.
    DirectX::XMVECTOR vMin, vMax;

    vMin = vMax = DirectX::XMVector3TransformNormal( XMLoadFloat3( pPoints ), InverseR );

    for( UINT i = 1; i < Count; i++ )
    {
        DirectX::XMVECTOR Point = DirectX::XMVector3TransformNormal( XMLoadFloat3( ( DirectX::XMFLOAT3* )( ( BYTE* )pPoints + i * Stride ) ),
                                                   InverseR );

        vMin = DirectX::XMVectorMin( vMin, Point );
        vMax = DirectX::XMVectorMax( vMax, Point );
    }

    // Rotate the center into world space.
    DirectX::XMVECTOR Center = ( vMin + vMax ) * 0.5f;
    Center = DirectX::XMVector3TransformNormal( Center, R );

    // Store center, extents, and orientation.
    XMStoreFloat3( &pOut->Center, Center );
    XMStoreFloat3( &pOut->Extents, ( vMax - vMin ) * 0.5f );
    XMStoreFloat4( &pOut->Orientation, Orientation );

    return;
}



//-----------------------------------------------------------------------------
// Build a frustum from a persepective projection matrix.  The matrix may only
// contain a projection; any rotation, translation or scale will cause the
// constructed frustum to be incorrect.
//-----------------------------------------------------------------------------
VOID ComputeFrustumFromProjection( Frustum* pOut, DirectX::XMMATRIX* pProjection )
{
    XMASSERT( pOut );
    XMASSERT( pProjection );

    // Corners of the projection frustum in homogenous space.
    static DirectX::XMVECTOR HomogenousPoints[6] =
    {
        {  1.0f,  0.0f, 1.0f, 1.0f },   // right (at far plane)
        { -1.0f,  0.0f, 1.0f, 1.0f },   // left
        {  0.0f,  1.0f, 1.0f, 1.0f },   // top
        {  0.0f, -1.0f, 1.0f, 1.0f },   // bottom

        { 0.0f, 0.0f, 0.0f, 1.0f },     // near
        { 0.0f, 0.0f, 1.0f, 1.0f }      // far
    };

    DirectX::XMVECTOR Determinant;
    DirectX::XMMATRIX matInverse = DirectX::XMMatrixInverse( &Determinant, *pProjection );

    // Compute the frustum corners in world space.
    DirectX::XMVECTOR Points[6];

    for( INT i = 0; i < 6; i++ )
    {
        // Transform point.
        Points[i] = DirectX::XMVector4Transform( HomogenousPoints[i], matInverse );
    }

    pOut->Origin = DirectX::XMFLOAT3( 0.0f, 0.0f, 0.0f );
    pOut->Orientation = DirectX::XMFLOAT4( 0.0f, 0.0f, 0.0f, 1.0f );

    // Compute the slopes.
    Points[0] = Points[0] * DirectX::XMVectorReciprocal( DirectX::XMVectorSplatZ( Points[0] ) );
    Points[1] = Points[1] * DirectX::XMVectorReciprocal( DirectX::XMVectorSplatZ( Points[1] ) );
    Points[2] = Points[2] * DirectX::XMVectorReciprocal( DirectX::XMVectorSplatZ( Points[2] ) );
    Points[3] = Points[3] * DirectX::XMVectorReciprocal( DirectX::XMVectorSplatZ( Points[3] ) );

    pOut->RightSlope = DirectX::XMVectorGetX( Points[0] );
    pOut->LeftSlope = DirectX::XMVectorGetX( Points[1] );
    pOut->TopSlope = DirectX::XMVectorGetY( Points[2] );
    pOut->BottomSlope = DirectX::XMVectorGetY( Points[3] );

    // Compute near and far.
    Points[4] = Points[4] * DirectX::XMVectorReciprocal( DirectX::XMVectorSplatW( Points[4] ) );
    Points[5] = Points[5] * DirectX::XMVectorReciprocal( DirectX::XMVectorSplatW( Points[5] ) );

    pOut->Near = DirectX::XMVectorGetZ( Points[4] );
    pOut->Far = DirectX::XMVectorGetZ( Points[5] );

    return;
}



//-----------------------------------------------------------------------------
// Build the 6 frustum planes from a frustum.
//-----------------------------------------------------------------------------
VOID ComputePlanesFromFrustum( const Frustum* pVolume, DirectX::XMVECTOR* pPlane0, DirectX::XMVECTOR* pPlane1, DirectX::XMVECTOR* pPlane2,
                               DirectX::XMVECTOR* pPlane3, DirectX::XMVECTOR* pPlane4, DirectX::XMVECTOR* pPlane5 )
{
    XMASSERT( pVolume );
    XMASSERT( pPlane0 );
    XMASSERT( pPlane1 );
    XMASSERT( pPlane2 );
    XMASSERT( pPlane3 );
    XMASSERT( pPlane4 );
    XMASSERT( pPlane5 );

    // Load origin and orientation of the frustum.
    DirectX::XMVECTOR Origin = XMLoadFloat3( &pVolume->Origin );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolume->Orientation );

    // Build the frustum planes.
    DirectX::XMVECTOR Plane0 = DirectX::XMVectorSet( 0.0f, 0.0f, -1.0f, pVolume->Near );
    DirectX::XMVECTOR Plane1 = DirectX::XMVectorSet( 0.0f, 0.0f, 1.0f, -pVolume->Far );
    DirectX::XMVECTOR Plane2 = DirectX::XMVectorSet( 1.0f, 0.0f, -pVolume->RightSlope, 0.0f );
    DirectX::XMVECTOR Plane3 = DirectX::XMVectorSet( -1.0f, 0.0f, pVolume->LeftSlope, 0.0f );
    DirectX::XMVECTOR Plane4 = DirectX::XMVectorSet( 0.0f, 1.0f, -pVolume->TopSlope, 0.0f );
    DirectX::XMVECTOR Plane5 = DirectX::XMVectorSet( 0.0f, -1.0f, pVolume->BottomSlope, 0.0f );

    Plane0 = TransformPlane( Plane0, Orientation, Origin );
    Plane1 = TransformPlane( Plane1, Orientation, Origin );
    Plane2 = TransformPlane( Plane2, Orientation, Origin );
    Plane3 = TransformPlane( Plane3, Orientation, Origin );
    Plane4 = TransformPlane( Plane4, Orientation, Origin );
    Plane5 = TransformPlane( Plane5, Orientation, Origin );

    *pPlane0 = XMPlaneNormalize( Plane0 );
    *pPlane1 = XMPlaneNormalize( Plane1 );
    *pPlane2 = XMPlaneNormalize( Plane2 );
    *pPlane3 = XMPlaneNormalize( Plane3 );
    *pPlane4 = XMPlaneNormalize( Plane4 );
    *pPlane5 = XMPlaneNormalize( Plane5 );
}



//-----------------------------------------------------------------------------
// Transform a sphere by an angle preserving transform.
//-----------------------------------------------------------------------------
VOID TransformSphere( Sphere* pOut, const Sphere* pIn, FLOAT Scale, DirectX::FXMVECTOR Rotation, DirectX::FXMVECTOR Translation )
{
    XMASSERT( pOut );
    XMASSERT( pIn );
    XMASSERT( XMQuaternionIsUnit( Rotation ) );

    // Load the center of the sphere.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pIn->Center );

    // Transform the center of the sphere.
    Center = DirectX::XMVector3Rotate( Center * DirectX::XMVectorReplicate( Scale ), Rotation ) + Translation;

    // Store the center sphere.
    XMStoreFloat3( &pOut->Center, Center );

    // Scale the radius of the pshere.
    pOut->Radius = pIn->Radius * Scale;

    return;
}



//-----------------------------------------------------------------------------
// Transform an axis aligned box by an angle preserving transform.
//-----------------------------------------------------------------------------
VOID TransformAxisAlignedBox( AxisAlignedBox* pOut, const AxisAlignedBox* pIn, FLOAT Scale, DirectX::FXMVECTOR Rotation,
                              FXMVECTOR Translation )
{
    XMASSERT( pOut );
    XMASSERT( pIn );
    XMASSERT( XMQuaternionIsUnit( Rotation ) );

    static DirectX::XMVECTOR Offset[8] =
    {
        { -1.0f, -1.0f, -1.0f, 0.0f },
        { -1.0f, -1.0f,  1.0f, 0.0f },
        { -1.0f,  1.0f, -1.0f, 0.0f },
        { -1.0f,  1.0f,  1.0f, 0.0f },
        {  1.0f, -1.0f, -1.0f, 0.0f },
        {  1.0f, -1.0f,  1.0f, 0.0f },
        {  1.0f,  1.0f, -1.0f, 0.0f },
        {  1.0f,  1.0f,  1.0f, 0.0f }
    };

    // Load center and extents.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pIn->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pIn->Extents );

    DirectX::XMVECTOR VectorScale = DirectX::XMVectorReplicate( Scale );

    // Compute and transform the corners and find new min/max bounds.
    DirectX::XMVECTOR Corner = Center + Extents * Offset[0];
    Corner = DirectX::XMVector3Rotate( Corner * VectorScale, Rotation ) + Translation;

    DirectX::XMVECTOR Min, Max;
    Min = Max = Corner;

    for( INT i = 1; i < 8; i++ )
    {
        Corner = Center + Extents * Offset[i];
        Corner = DirectX::XMVector3Rotate( Corner * VectorScale, Rotation ) + Translation;

        Min = DirectX::XMVectorMin( Min, Corner );
        Max = DirectX::XMVectorMax( Max, Corner );
    }

    // Store center and extents.
    XMStoreFloat3( &pOut->Center, ( Min + Max ) * 0.5f );
    XMStoreFloat3( &pOut->Extents, ( Max - Min ) * 0.5f );

    return;
}



//-----------------------------------------------------------------------------
// Transform an oriented box by an angle preserving transform.
//-----------------------------------------------------------------------------
VOID TransformOrientedBox( OrientedBox* pOut, const OrientedBox* pIn, FLOAT Scale, DirectX::FXMVECTOR Rotation,
                           FXMVECTOR Translation )
{
    XMASSERT( pOut );
    XMASSERT( pIn );
    XMASSERT( XMQuaternionIsUnit( Rotation ) );

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pIn->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pIn->Extents );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pIn->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Composite the box rotation and the transform rotation.
    Orientation = XMQuaternionMultiply( Orientation, Rotation );

    // Transform the center.
    DirectX::XMVECTOR VectorScale = DirectX::XMVectorReplicate( Scale );
    Center = DirectX::XMVector3Rotate( Center * VectorScale, Rotation ) + Translation;

    // Scale the box extents.
    Extents = Extents * VectorScale;

    // Store the box.
    XMStoreFloat3( &pOut->Center, Center );
    XMStoreFloat3( &pOut->Extents, Extents );
    XMStoreFloat4( &pOut->Orientation, Orientation );

    return;
}



//-----------------------------------------------------------------------------
// Transform a frustum by an angle preserving transform.
//-----------------------------------------------------------------------------
VOID TransformFrustum( Frustum* pOut, const Frustum* pIn, FLOAT Scale, DirectX::FXMVECTOR Rotation, DirectX::FXMVECTOR Translation )
{
    XMASSERT( pOut );
    XMASSERT( pIn );
    XMASSERT( XMQuaternionIsUnit( Rotation ) );

    // Load the frustum.
    DirectX::XMVECTOR Origin = XMLoadFloat3( &pIn->Origin );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pIn->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Composite the frustum rotation and the transform rotation.
    Orientation = XMQuaternionMultiply( Orientation, Rotation );

    // Transform the origin.
    Origin = DirectX::XMVector3Rotate( Origin * DirectX::XMVectorReplicate( Scale ), Rotation ) + Translation;

    // Store the frustum.
    XMStoreFloat3( &pOut->Origin, Origin );
    XMStoreFloat4( &pOut->Orientation, Orientation );

    // Scale the near and far distances (the slopes remain the same).
    pOut->Near = pIn->Near * Scale;
    pOut->Far = pIn->Far * Scale;

    // Copy the slopes.
    pOut->RightSlope = pIn->RightSlope;
    pOut->LeftSlope = pIn->LeftSlope;
    pOut->TopSlope = pIn->TopSlope;
    pOut->BottomSlope = pIn->BottomSlope;

    return;
}



//-----------------------------------------------------------------------------
// Point in sphere test.
//-----------------------------------------------------------------------------
BOOL IntersectPointSphere( FXMVECTOR Point, const Sphere* pVolume )
{
    XMASSERT( pVolume );

    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Radius = DirectX::XMVectorReplicatePtr( &pVolume->Radius );

    DirectX::XMVECTOR DistanceSquared = DirectX::XMVector3LengthSq( Point - Center );
    DirectX::XMVECTOR RadiusSquared = Radius * Radius;

    return DirectX::XMVector4LessOrEqual( DistanceSquared, RadiusSquared );
}



//-----------------------------------------------------------------------------
// Point in axis aligned box test.
//-----------------------------------------------------------------------------
BOOL IntersectPointAxisAlignedBox( FXMVECTOR Point, const AxisAlignedBox* pVolume )
{
    XMASSERT( pVolume );

    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );

    return DirectX::XMVector3InBounds( Point - Center, Extents );
}



//-----------------------------------------------------------------------------
// Point in oriented box test.
//-----------------------------------------------------------------------------
BOOL IntersectPointOrientedBox( FXMVECTOR Point, const OrientedBox* pVolume )
{
    XMASSERT( pVolume );

    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Transform the point to be local to the box.
    DirectX::XMVECTOR TPoint = DirectX::XMVector3InverseRotate( Point - Center, Orientation );

    return DirectX::XMVector3InBounds( TPoint, Extents );
}



//-----------------------------------------------------------------------------
// Point in frustum test.
//-----------------------------------------------------------------------------
BOOL IntersectPointFrustum( FXMVECTOR Point, const Frustum* pVolume )
{
    static const XMVECTORU32 SelectW = {DirectX::XM_SELECT_0, DirectX::XM_SELECT_0, DirectX::XM_SELECT_0, DirectX::XM_SELECT_1};
    static const XMVECTORU32 SelectZ = {DirectX::XM_SELECT_0, DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_0};

    static const DirectX::XMVECTOR BasePlanes[6] = 
    {
        {  0.0f,  0.0f, -1.0f, 0.0f },
        {  0.0f,  0.0f,  1.0f, 0.0f },
        {  1.0f,  0.0f,  0.0f, 0.0f },
        { -1.0f,  0.0f,  0.0f, 0.0f },
        {  0.0f,  1.0f,  0.0f, 0.0f },
        {  0.0f, -1.0f,  0.0f, 0.0f },
    };

    XMASSERT( pVolume );

    // Build frustum planes.
    DirectX::XMVECTOR Planes[6];
    Planes[0] = DirectX::XMVectorSelect( BasePlanes[0], DirectX::XMVectorSplatX(  XMLoadFloat( &pVolume->Near ) ),
                                SelectW );
    Planes[1] = DirectX::XMVectorSelect( BasePlanes[1], DirectX::XMVectorSplatX( -XMLoadFloat( &pVolume->Far ) ),
                                SelectW );
    Planes[2] = DirectX::XMVectorSelect( BasePlanes[2], DirectX::XMVectorSplatX( -XMLoadFloat( &pVolume->RightSlope ) ),
                                SelectZ );
    Planes[3] = DirectX::XMVectorSelect( BasePlanes[3], DirectX::XMVectorSplatX(  XMLoadFloat( &pVolume->LeftSlope ) ),
                                SelectZ );
    Planes[4] = DirectX::XMVectorSelect( BasePlanes[4], DirectX::XMVectorSplatX( -XMLoadFloat( &pVolume->TopSlope ) ),
                                SelectZ );
    Planes[5] = DirectX::XMVectorSelect( BasePlanes[5], DirectX::XMVectorSplatX(  XMLoadFloat( &pVolume->BottomSlope ) ),
                                SelectZ );

    // Load origin and orientation.
    DirectX::XMVECTOR Origin = XMLoadFloat3( &pVolume->Origin );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Transform point into local space of frustum.
    DirectX::XMVECTOR TPoint = DirectX::XMVector3InverseRotate( Point - Origin, Orientation );

    // Set w to one.
    TPoint = DirectX::XMVectorInsert( TPoint, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1);

    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();
    DirectX::XMVECTOR Outside = Zero;

    // Test point against each plane of the frustum.
    for( INT i = 0; i < 6; i++ )
    {
        DirectX::XMVECTOR Dot = DirectX::XMVector4Dot( TPoint, Planes[i] );
        Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( Dot, Zero ) );
    }

    return DirectX::XMVector4NotEqualInt( Outside, DirectX::XMVectorTrueInt() );
}



//-----------------------------------------------------------------------------
// Compute the intersection of a ray (Origin, Direction) with a triangle 
// (V0, V1, V2).  Return TRUE if there is an intersection and also set *pDist 
// to the distance along the ray to the intersection.
// 
// The algorithm is based on Moller, Tomas and Trumbore, "Fast, Minimum Storage 
// Ray-Triangle Intersection", Journal of Graphics Tools, vol. 2, no. 1, 
// pp 21-28, 1997.
//-----------------------------------------------------------------------------
BOOL IntersectRayTriangle( FXMVECTOR Origin, DirectX::FXMVECTOR Direction, DirectX::FXMVECTOR V0, CXMVECTOR V1, CXMVECTOR V2,
                           FLOAT* pDist )
{
    XMASSERT( pDist );
    XMASSERT( DirectX::XMVector3IsUnit( Direction ) );

    static const DirectX::XMVECTOR Epsilon =
    {
        1e-20f, 1e-20f, 1e-20f, 1e-20f
    };

    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();

    DirectX::XMVECTOR e1 = V1 - V0;
    DirectX::XMVECTOR e2 = V2 - V0;

    // p = Direction ^ e2;
    DirectX::XMVECTOR p = DirectX::XMVector3Cross( Direction, e2 );

    // det = e1 * p;
    DirectX::XMVECTOR det = DirectX::XMVector3Dot( e1, p );

    DirectX::XMVECTOR u, v, t;

    if( DirectX::XMVector3GreaterOrEqual( det, Epsilon ) )
    {
        // Determinate is positive (front side of the triangle).
        DirectX::XMVECTOR s = Origin - V0;

        // u = s * p;
        u = DirectX::XMVector3Dot( s, p );

        DirectX::XMVECTOR NoIntersection = DirectX::XMVectorLess( u, Zero );
        NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( u, det ) );

        // q = s ^ e1;
        DirectX::XMVECTOR q = DirectX::XMVector3Cross( s, e1 );

        // v = Direction * q;
        v = DirectX::XMVector3Dot( Direction, q );

        NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( v, Zero ) );
        NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( u + v, det ) );

        // t = e2 * q;
        t = DirectX::XMVector3Dot( e2, q );

        NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( t, Zero ) );

        if( DirectX::XMVector4EqualInt( NoIntersection, DirectX::XMVectorTrueInt() ) )
            return FALSE;
    }
    else if( DirectX::XMVector3LessOrEqual( det, -Epsilon ) )
    {
        // Determinate is negative (back side of the triangle).
        DirectX::XMVECTOR s = Origin - V0;

        // u = s * p;
        u = DirectX::XMVector3Dot( s, p );

        DirectX::XMVECTOR NoIntersection = DirectX::XMVectorGreater( u, Zero );
        NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( u, det ) );

        // q = s ^ e1;
        DirectX::XMVECTOR q = DirectX::XMVector3Cross( s, e1 );

        // v = Direction * q;
        v = DirectX::XMVector3Dot( Direction, q );

        NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( v, Zero ) );
        NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( u + v, det ) );

        // t = e2 * q;
        t = DirectX::XMVector3Dot( e2, q );

        NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( t, Zero ) );

        if ( DirectX::XMVector4EqualInt( NoIntersection, DirectX::XMVectorTrueInt() ) )
            return FALSE;
    }
    else
    {
        // Parallel ray.
        return FALSE;
    }

    DirectX::XMVECTOR inv_det = DirectX::XMVectorReciprocal( det );

    t *= inv_det;

    // u * inv_det and v * inv_det are the barycentric cooridinates of the intersection.

    // Store the x-component to *pDist
    XMStoreFloat( pDist, t );

    return TRUE;
}



//-----------------------------------------------------------------------------
// Compute the intersection of a ray (Origin, Direction) with a sphere.
//-----------------------------------------------------------------------------
BOOL IntersectRaySphere( FXMVECTOR Origin, DirectX::FXMVECTOR Direction, const Sphere* pVolume, FLOAT* pDist )
{
    XMASSERT( pVolume );
    XMASSERT( pDist );
    XMASSERT( DirectX::XMVector3IsUnit( Direction ) );

    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Radius = DirectX::XMVectorReplicatePtr( &pVolume->Radius );

    // l is the vector from the ray origin to the center of the sphere.
    DirectX::XMVECTOR l = Center - Origin;

    // s is the projection of the l onto the ray direction.
    DirectX::XMVECTOR s = DirectX::XMVector3Dot( l, Direction );

    DirectX::XMVECTOR l2 = DirectX::XMVector3Dot( l, l );

    DirectX::XMVECTOR r2 = Radius * Radius;

    // m2 is squared distance from the center of the sphere to the projection.
    DirectX::XMVECTOR m2 = l2 - s * s;

    DirectX::XMVECTOR NoIntersection;

    // If the ray origin is outside the sphere and the center of the sphere is 
    // behind the ray origin there is no intersection.
    NoIntersection = DirectX::XMVectorAndInt( DirectX::XMVectorLess( s, DirectX::XMVectorZero() ), DirectX::XMVectorGreater( l2, r2 ) );

    // If the squared distance from the center of the sphere to the projection
    // is greater than the radius squared the ray will miss the sphere.
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( m2, r2 ) );

    // The ray hits the sphere, compute the nearest intersection point.
    DirectX::XMVECTOR q = DirectX::XMVectorSqrt( r2 - m2 );
    DirectX::XMVECTOR t1 = s - q;
    DirectX::XMVECTOR t2 = s + q;

    DirectX::XMVECTOR OriginInside = DirectX::XMVectorLessOrEqual( l2, r2 );
    DirectX::XMVECTOR t = DirectX::XMVectorSelect( t1, t2, OriginInside );

    if( DirectX::XMVector4NotEqualInt( NoIntersection, DirectX::XMVectorTrueInt() ) )
    {
        // Store the x-component to *pDist.
        XMStoreFloat( pDist, t );
        return TRUE;
    }

    return FALSE;
}



//-----------------------------------------------------------------------------
// Compute the intersection of a ray (Origin, Direction) with an axis aligned 
// box using the slabs method.
//-----------------------------------------------------------------------------
BOOL IntersectRayAxisAlignedBox( FXMVECTOR Origin, DirectX::FXMVECTOR Direction, const AxisAlignedBox* pVolume, FLOAT* pDist )
{
    XMASSERT( pVolume );
    XMASSERT( pDist );
    XMASSERT( DirectX::XMVector3IsUnit( Direction ) );

    static const DirectX::XMVECTOR Epsilon =
    {
        1e-20f, 1e-20f, 1e-20f, 1e-20f
    };
    static const DirectX::XMVECTOR FltMin =
    {
        -FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX
    };
    static const DirectX::XMVECTOR FltMax =
    {
        FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX
    };

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );

    // Adjust ray origin to be relative to center of the box.
    DirectX::XMVECTOR TOrigin = Center - Origin;

    // Compute the dot product againt each axis of the box.
    // Since the axii are (1,0,0), (0,1,0), (0,0,1) no computation is necessary.
    DirectX::XMVECTOR AxisDotOrigin = TOrigin;
    DirectX::XMVECTOR AxisDotDirection = Direction;

    // if (fabs(AxisDotDirection) <= Epsilon) the ray is nearly parallel to the slab.
    DirectX::XMVECTOR IsParallel = DirectX::XMVectorLessOrEqual( DirectX::XMVectorAbs( AxisDotDirection ), Epsilon );

    // Test against all three axii simultaneously.
    DirectX::XMVECTOR InverseAxisDotDirection = DirectX::XMVectorReciprocal( AxisDotDirection );
    DirectX::XMVECTOR t1 = ( AxisDotOrigin - Extents ) * InverseAxisDotDirection;
    DirectX::XMVECTOR t2 = ( AxisDotOrigin + Extents ) * InverseAxisDotDirection;

    // Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
    // use the results from any directions parallel to the slab.
    DirectX::XMVECTOR t_min = DirectX::XMVectorSelect( DirectX::XMVectorMin( t1, t2 ), FltMin, IsParallel );
    DirectX::XMVECTOR t_max = DirectX::XMVectorSelect( DirectX::XMVectorMax( t1, t2 ), FltMax, IsParallel );

    // t_min.x = maximum( t_min.x, t_min.y, t_min.z );
    // t_max.x = minimum( t_max.x, t_max.y, t_max.z );
    t_min = DirectX::XMVectorMax( t_min, DirectX::XMVectorSplatY( t_min ) );  // x = max(x,y)
    t_min = DirectX::XMVectorMax( t_min, DirectX::XMVectorSplatZ( t_min ) );  // x = max(max(x,y),z)
    t_max = DirectX::XMVectorMin( t_max, DirectX::XMVectorSplatY( t_max ) );  // x = min(x,y)
    t_max = DirectX::XMVectorMin( t_max, DirectX::XMVectorSplatZ( t_max ) );  // x = min(min(x,y),z)

    // if ( t_min > t_max ) return FALSE;
    DirectX::XMVECTOR NoIntersection = DirectX::XMVectorGreater( DirectX::XMVectorSplatX( t_min ), DirectX::XMVectorSplatX( t_max ) );

    // if ( t_max < 0.0f ) return FALSE;
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( DirectX::XMVectorSplatX( t_max ), DirectX::XMVectorZero() ) );

    // if (IsParallel && (-Extents > AxisDotOrigin || Extents < AxisDotOrigin)) return FALSE;
    DirectX::XMVECTOR ParallelOverlap = DirectX::XMVectorInBounds( AxisDotOrigin, Extents );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorAndCInt( IsParallel, ParallelOverlap ) );

    if( !DirectX::XMVector3AnyTrue( NoIntersection ) )
    {
        // Store the x-component to *pDist
        XMStoreFloat( pDist, t_min );
        return TRUE;
    }

    return FALSE;
}



//-----------------------------------------------------------------------------
// Compute the intersection of a ray (Origin, Direction) with an oriented box
// using the slabs method.
//-----------------------------------------------------------------------------
BOOL IntersectRayOrientedBox( FXMVECTOR Origin, DirectX::FXMVECTOR Direction, const OrientedBox* pVolume, FLOAT* pDist )
{
    XMASSERT( pVolume );
    XMASSERT( pDist );
    XMASSERT( DirectX::XMVector3IsUnit( Direction ) );

    static const DirectX::XMVECTOR Epsilon =
    {
        1e-20f, 1e-20f, 1e-20f, 1e-20f
    };
    static const DirectX::XMVECTOR FltMin =
    {
        -FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX
    };
    static const DirectX::XMVECTOR FltMax =
    {
        FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX
    };
    static const XMVECTORI32 SelectY =
    {
        DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_0, DirectX::XM_SELECT_0
    };
    static const XMVECTORI32 SelectZ =
    {
        DirectX::XM_SELECT_0, DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_0
    };

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Get the boxes normalized side directions.
    DirectX::XMMATRIX R = DirectX::XMMatrixRotationQuaternion( Orientation );

    // Adjust ray origin to be relative to center of the box.
    DirectX::XMVECTOR TOrigin = Center - Origin;

    // Compute the dot product againt each axis of the box.
    DirectX::XMVECTOR AxisDotOrigin = DirectX::XMVector3Dot( R.r[0], TOrigin );
    AxisDotOrigin = DirectX::XMVectorSelect( AxisDotOrigin, DirectX::XMVector3Dot( R.r[1], TOrigin ), SelectY );
    AxisDotOrigin = DirectX::XMVectorSelect( AxisDotOrigin, DirectX::XMVector3Dot( R.r[2], TOrigin ), SelectZ );

    DirectX::XMVECTOR AxisDotDirection = DirectX::XMVector3Dot( R.r[0], Direction );
    AxisDotDirection = DirectX::XMVectorSelect( AxisDotDirection, DirectX::XMVector3Dot( R.r[1], Direction ), SelectY );
    AxisDotDirection = DirectX::XMVectorSelect( AxisDotDirection, DirectX::XMVector3Dot( R.r[2], Direction ), SelectZ );

    // if (fabs(AxisDotDirection) <= Epsilon) the ray is nearly parallel to the slab.
    DirectX::XMVECTOR IsParallel = DirectX::XMVectorLessOrEqual( DirectX::XMVectorAbs( AxisDotDirection ), Epsilon );

    // Test against all three axes simultaneously.
    DirectX::XMVECTOR InverseAxisDotDirection = DirectX::XMVectorReciprocal( AxisDotDirection );
    DirectX::XMVECTOR t1 = ( AxisDotOrigin - Extents ) * InverseAxisDotDirection;
    DirectX::XMVECTOR t2 = ( AxisDotOrigin + Extents ) * InverseAxisDotDirection;

    // Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
    // use the results from any directions parallel to the slab.
    DirectX::XMVECTOR t_min = DirectX::XMVectorSelect( DirectX::XMVectorMin( t1, t2 ), FltMin, IsParallel );
    DirectX::XMVECTOR t_max = DirectX::XMVectorSelect( DirectX::XMVectorMax( t1, t2 ), FltMax, IsParallel );

    // t_min.x = maximum( t_min.x, t_min.y, t_min.z );
    // t_max.x = minimum( t_max.x, t_max.y, t_max.z );
    t_min = DirectX::XMVectorMax( t_min, DirectX::XMVectorSplatY( t_min ) );  // x = max(x,y)
    t_min = DirectX::XMVectorMax( t_min, DirectX::XMVectorSplatZ( t_min ) );  // x = max(max(x,y),z)
    t_max = DirectX::XMVectorMin( t_max, DirectX::XMVectorSplatY( t_max ) );  // x = min(x,y)
    t_max = DirectX::XMVectorMin( t_max, DirectX::XMVectorSplatZ( t_max ) );  // x = min(min(x,y),z)

    // if ( t_min > t_max ) return FALSE;
    DirectX::XMVECTOR NoIntersection = DirectX::XMVectorGreater( DirectX::XMVectorSplatX( t_min ), DirectX::XMVectorSplatX( t_max ) );

    // if ( t_max < 0.0f ) return FALSE;
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( DirectX::XMVectorSplatX( t_max ), DirectX::XMVectorZero() ) );

    // if (IsParallel && (-Extents > AxisDotOrigin || Extents < AxisDotOrigin)) return FALSE;
    DirectX::XMVECTOR ParallelOverlap = DirectX::XMVectorInBounds( AxisDotOrigin, Extents );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorAndCInt( IsParallel, ParallelOverlap ) );

    if( !DirectX::XMVector3AnyTrue( NoIntersection ) )
    {
        // Store the x-component to *pDist
        XMStoreFloat( pDist, t_min );
        return TRUE;
    }

    return FALSE;
}



//-----------------------------------------------------------------------------
// Test if two triangles intersect.
//
// The final test of algorithm is based on Shen, Heng, and Tang, "A Fast 
// Triangle-Triangle Overlap Test Using Signed Distances", Journal of Graphics 
// Tools, vol. 8, no. 1, pp 17-23, 2003 and Guigue and Devillers, "Fast and 
// Robust Triangle-Triangle Overlap Test Using Orientation Predicates", Journal 
// of Graphics Tools, vol. 8, no. 1, pp 25-32, 2003.
//
// The final test could be considered an edge-edge separating plane test with
// the 9 possible cases narrowed down to the only two pairs of edges that can 
// actaully result in a seperation.
//-----------------------------------------------------------------------------
BOOL IntersectTriangleTriangle( FXMVECTOR A0, DirectX::FXMVECTOR A1, DirectX::FXMVECTOR A2, CXMVECTOR B0, CXMVECTOR B1, CXMVECTOR B2 )
{
    static const DirectX::XMVECTOR Epsilon =
    {
        1e-20f, 1e-20f, 1e-20f, 1e-20f
    };
    static const XMVECTORI32 SelectY =
    {
        DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_0, DirectX::XM_SELECT_0
    };
    static const XMVECTORI32 SelectZ =
    {
        DirectX::XM_SELECT_0, DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_0
    };
    static const XMVECTORI32 Select0111 =
    {
        DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_1, DirectX::XM_SELECT_1
    };
    static const XMVECTORI32 Select1011 =
    {
        DirectX::XM_SELECT_1, DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_1
    };
    static const XMVECTORI32 Select1101 =
    {
        DirectX::XM_SELECT_1, DirectX::XM_SELECT_1, DirectX::XM_SELECT_0, DirectX::XM_SELECT_1
    };

    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();

    // Compute the normal of triangle A.
    DirectX::XMVECTOR N1 = DirectX::XMVector3Cross( A1 - A0, A2 - A0 );

    // Assert that the triangle is not degenerate.
    XMASSERT( !DirectX::XMVector3Equal( N1, Zero ) );

    // Test points of B against the plane of A.
    DirectX::XMVECTOR BDist = DirectX::XMVector3Dot( N1, B0 - A0 );
    BDist = DirectX::XMVectorSelect( BDist, DirectX::XMVector3Dot( N1, B1 - A0 ), SelectY );
    BDist = DirectX::XMVectorSelect( BDist, DirectX::XMVector3Dot( N1, B2 - A0 ), SelectZ );

    // Ensure robustness with co-planar triangles by zeroing small distances.
    UINT BDistIsZeroCR;
    DirectX::XMVECTOR BDistIsZero = DirectX::XMVectorGreaterR( &BDistIsZeroCR, Epsilon, DirectX::XMVectorAbs( BDist ) );
    BDist = DirectX::XMVectorSelect( BDist, Zero, BDistIsZero );

    UINT BDistIsLessCR;
    DirectX::XMVECTOR BDistIsLess = DirectX::XMVectorGreaterR( &BDistIsLessCR, Zero, BDist );

    UINT BDistIsGreaterCR;
    DirectX::XMVECTOR BDistIsGreater = DirectX::XMVectorGreaterR( &BDistIsGreaterCR, BDist, Zero );

    // If all the points are on the same side we don't intersect.
    if( XMComparisonAllTrue( BDistIsLessCR ) || XMComparisonAllTrue( BDistIsGreaterCR ) )
        return FALSE;

    // Compute the normal of triangle B.
    DirectX::XMVECTOR N2 = DirectX::XMVector3Cross( B1 - B0, B2 - B0 );

    // Assert that the triangle is not degenerate.
    XMASSERT( !DirectX::XMVector3Equal( N2, Zero ) );

    // Test points of A against the plane of B.
    DirectX::XMVECTOR ADist = DirectX::XMVector3Dot( N2, A0 - B0 );
    ADist = DirectX::XMVectorSelect( ADist, DirectX::XMVector3Dot( N2, A1 - B0 ), SelectY );
    ADist = DirectX::XMVectorSelect( ADist, DirectX::XMVector3Dot( N2, A2 - B0 ), SelectZ );

    // Ensure robustness with co-planar triangles by zeroing small distances.
    UINT ADistIsZeroCR;
    DirectX::XMVECTOR ADistIsZero = DirectX::XMVectorGreaterR( &ADistIsZeroCR, Epsilon, DirectX::XMVectorAbs( BDist ) );
    ADist = DirectX::XMVectorSelect( ADist, Zero, ADistIsZero );

    UINT ADistIsLessCR;
    DirectX::XMVECTOR ADistIsLess = DirectX::XMVectorGreaterR( &ADistIsLessCR, Zero, ADist );

    UINT ADistIsGreaterCR;
    DirectX::XMVECTOR ADistIsGreater = DirectX::XMVectorGreaterR( &ADistIsGreaterCR, ADist, Zero );

    // If all the points are on the same side we don't intersect.
    if( XMComparisonAllTrue( ADistIsLessCR ) || XMComparisonAllTrue( ADistIsGreaterCR ) )
        return FALSE;

    // Special case for co-planar triangles.
    if( XMComparisonAllTrue( ADistIsZeroCR ) || XMComparisonAllTrue( BDistIsZeroCR ) )
    {
        DirectX::XMVECTOR Axis, Dist, MinDist;

        // Compute an axis perpindicular to the edge (points out).
        Axis = DirectX::XMVector3Cross( N1, A1 - A0 );
        Dist = DirectX::XMVector3Dot( Axis, A0 );

        // Test points of B against the axis.
        MinDist = DirectX::XMVector3Dot( B0, Axis );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( B1, Axis ) );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( B2, Axis ) );
        if( DirectX::XMVector4GreaterOrEqual( MinDist, Dist ) )
            return FALSE;

        // Edge (A1, A2)
        Axis = DirectX::XMVector3Cross( N1, A2 - A1 );
        Dist = DirectX::XMVector3Dot( Axis, A1 );

        MinDist = DirectX::XMVector3Dot( B0, Axis );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( B1, Axis ) );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( B2, Axis ) );
        if( DirectX::XMVector4GreaterOrEqual( MinDist, Dist ) )
            return FALSE;

        // Edge (A2, A0)
        Axis = DirectX::XMVector3Cross( N1, A0 - A2 );
        Dist = DirectX::XMVector3Dot( Axis, A2 );

        MinDist = DirectX::XMVector3Dot( B0, Axis );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( B1, Axis ) );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( B2, Axis ) );
        if( DirectX::XMVector4GreaterOrEqual( MinDist, Dist ) )
            return FALSE;

        // Edge (B0, B1)
        Axis = DirectX::XMVector3Cross( N2, B1 - B0 );
        Dist = DirectX::XMVector3Dot( Axis, B0 );

        MinDist = DirectX::XMVector3Dot( A0, Axis );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( A1, Axis ) );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( A2, Axis ) );
        if( DirectX::XMVector4GreaterOrEqual( MinDist, Dist ) )
            return FALSE;

        // Edge (B1, B2)
        Axis = DirectX::XMVector3Cross( N2, B2 - B1 );
        Dist = DirectX::XMVector3Dot( Axis, B1 );

        MinDist = DirectX::XMVector3Dot( A0, Axis );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( A1, Axis ) );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( A2, Axis ) );
        if( DirectX::XMVector4GreaterOrEqual( MinDist, Dist ) )
            return FALSE;

        // Edge (B2,B0)
        Axis = DirectX::XMVector3Cross( N2, B0 - B2 );
        Dist = DirectX::XMVector3Dot( Axis, B2 );

        MinDist = DirectX::XMVector3Dot( A0, Axis );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( A1, Axis ) );
        MinDist = DirectX::XMVectorMin( MinDist, DirectX::XMVector3Dot( A2, Axis ) );
        if( DirectX::XMVector4GreaterOrEqual( MinDist, Dist ) )
            return FALSE;

        return TRUE;
    }

    //
    // Find the single vertex of A and B (ie the vertex on the opposite side
    // of the plane from the other two) and reorder the edges so we can compute 
    // the signed edge/edge distances.
    //
    // if ( (V0 >= 0 && V1 <  0 && V2 <  0) ||
    //      (V0 >  0 && V1 <= 0 && V2 <= 0) ||
    //      (V0 <= 0 && V1 >  0 && V2 >  0) ||
    //      (V0 <  0 && V1 >= 0 && V2 >= 0) ) then V0 is singular;
    //
    // If our singular vertex is not on the positive side of the plane we reverse
    // the triangle winding so that the overlap comparisons will compare the 
    // correct edges with the correct signs.
    //
    DirectX::XMVECTOR ADistIsLessEqual = DirectX::XMVectorOrInt( ADistIsLess, ADistIsZero );
    DirectX::XMVECTOR ADistIsGreaterEqual = DirectX::XMVectorOrInt( ADistIsGreater, ADistIsZero );

    DirectX::XMVECTOR AA0, AA1, AA2;
    bool bPositiveA;

    if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsGreaterEqual, ADistIsLess, Select0111 ) ) ||
        DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsGreater, ADistIsLessEqual, Select0111 ) ) )
    {
        // A0 is singular, crossing from positive to negative.
        AA0 = A0; AA1 = A1; AA2 = A2;
        bPositiveA = true;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsLessEqual, ADistIsGreater, Select0111 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsLess, ADistIsGreaterEqual, Select0111 ) ) )
    {
        // A0 is singular, crossing from negative to positive.
        AA0 = A0; AA1 = A2; AA2 = A1;
        bPositiveA = false;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsGreaterEqual, ADistIsLess, Select1011 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsGreater, ADistIsLessEqual, Select1011 ) ) )
    {
        // A1 is singular, crossing from positive to negative.
        AA0 = A1; AA1 = A2; AA2 = A0;
        bPositiveA = true;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsLessEqual, ADistIsGreater, Select1011 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsLess, ADistIsGreaterEqual, Select1011 ) ) )
    {
        // A1 is singular, crossing from negative to positive.
        AA0 = A1; AA1 = A0; AA2 = A2;
        bPositiveA = false;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsGreaterEqual, ADistIsLess, Select1101 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsGreater, ADistIsLessEqual, Select1101 ) ) )
    {
        // A2 is singular, crossing from positive to negative.
        AA0 = A2; AA1 = A0; AA2 = A1;
        bPositiveA = true;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsLessEqual, ADistIsGreater, Select1101 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( ADistIsLess, ADistIsGreaterEqual, Select1101 ) ) )
    {
        // A2 is singular, crossing from negative to positive.
        AA0 = A2; AA1 = A1; AA2 = A0;
        bPositiveA = false;
    }
    else
    {
        XMASSERT( FALSE );
        return FALSE;
    }

    DirectX::XMVECTOR BDistIsLessEqual = DirectX::XMVectorOrInt( BDistIsLess, BDistIsZero );
    DirectX::XMVECTOR BDistIsGreaterEqual = DirectX::XMVectorOrInt( BDistIsGreater, BDistIsZero );

    DirectX::XMVECTOR BB0, BB1, BB2;
    bool bPositiveB;

    if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsGreaterEqual, BDistIsLess, Select0111 ) ) ||
        DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsGreater, BDistIsLessEqual, Select0111 ) ) )
    {
        // B0 is singular, crossing from positive to negative.
        BB0 = B0; BB1 = B1; BB2 = B2;
        bPositiveB = true;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsLessEqual, BDistIsGreater, Select0111 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsLess, BDistIsGreaterEqual, Select0111 ) ) )
    {
        // B0 is singular, crossing from negative to positive.
        BB0 = B0; BB1 = B2; BB2 = B1;
        bPositiveB = false;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsGreaterEqual, BDistIsLess, Select1011 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsGreater, BDistIsLessEqual, Select1011 ) ) )
    {
        // B1 is singular, crossing from positive to negative.
        BB0 = B1; BB1 = B2; BB2 = B0;
        bPositiveB = true;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsLessEqual, BDistIsGreater, Select1011 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsLess, BDistIsGreaterEqual, Select1011 ) ) )
    {
        // B1 is singular, crossing from negative to positive.
        BB0 = B1; BB1 = B0; BB2 = B2;
        bPositiveB = false;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsGreaterEqual, BDistIsLess, Select1101 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsGreater, BDistIsLessEqual, Select1101 ) ) )
    {
        // B2 is singular, crossing from positive to negative.
        BB0 = B2; BB1 = B0; BB2 = B1;
        bPositiveB = true;
    }
    else if( DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsLessEqual, BDistIsGreater, Select1101 ) ) ||
             DirectX::XMVector3AllTrue( DirectX::XMVectorSelect( BDistIsLess, BDistIsGreaterEqual, Select1101 ) ) )
    {
        // B2 is singular, crossing from negative to positive.
        BB0 = B2; BB1 = B1; BB2 = B0;
        bPositiveB = false;
    }
    else
    {
        XMASSERT( FALSE );
        return FALSE;
    }

    DirectX::XMVECTOR Delta0, Delta1;

    // Reverse the direction of the test depending on whether the singular vertices are
    // the same sign or different signs.
    if( bPositiveA ^ bPositiveB )
    {
        Delta0 = ( BB0 - AA0 );
        Delta1 = ( AA0 - BB0 );
    }
    else
    {
        Delta0 = ( AA0 - BB0 );
        Delta1 = ( BB0 - AA0 );
    }

    // Check if the triangles overlap on the line of intersection between the
    // planes of the two triangles by finding the signed line distances.
    DirectX::XMVECTOR Dist0 = DirectX::XMVector3Dot( Delta0, DirectX::XMVector3Cross( ( BB2 - BB0 ), ( AA2 - AA0 ) ) );
    if( DirectX::XMVector4Greater( Dist0, Zero ) )
        return FALSE;

    DirectX::XMVECTOR Dist1 = DirectX::XMVector3Dot( Delta1, DirectX::XMVector3Cross( ( BB1 - BB0 ), ( AA1 - AA0 ) ) );
    if( DirectX::XMVector4Greater( Dist1, Zero ) )
        return FALSE;

    return TRUE;
}



//-----------------------------------------------------------------------------
BOOL IntersectTriangleSphere( FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::FXMVECTOR V2, const Sphere* pVolume )
{
    XMASSERT( pVolume );

    // Load the sphere.    
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Radius = DirectX::XMVectorReplicatePtr( &pVolume->Radius );

    // Compute the plane of the triangle (has to be normalized).
    DirectX::XMVECTOR N = DirectX::XMVector3Normalize( DirectX::XMVector3Cross( V1 - V0, V2 - V0 ) );

    // Assert that the triangle is not degenerate.
    XMASSERT( !DirectX::XMVector3Equal( N, DirectX::XMVectorZero() ) );

    // Find the nearest feature on the triangle to the sphere.
    DirectX::XMVECTOR Dist = DirectX::XMVector3Dot( Center - V0, N );

    // If the center of the sphere is farther from the plane of the triangle than
    // the radius of the sphere, then there cannot be an intersection.
    DirectX::XMVECTOR NoIntersection = DirectX::XMVectorLess( Dist, -Radius );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Dist, Radius ) );

    // Project the center of the sphere onto the plane of the triangle.
    DirectX::XMVECTOR Point = Center - ( N * Dist );

    // Is it inside all the edges? If so we intersect because the distance 
    // to the plane is less than the radius.
    DirectX::XMVECTOR Intersection = PointOnPlaneInsideTriangle( Point, V0, V1, V2 );

    // Find the nearest point on each edge.
    DirectX::XMVECTOR RadiusSq = Radius * Radius;

    // Edge 0,1
    Point = PointOnLineSegmentNearestPoint( V0, V1, Center );

    // If the distance to the center of the sphere to the point is less than 
    // the radius of the sphere then it must intersect.
    Intersection = DirectX::XMVectorOrInt( Intersection, DirectX::XMVectorLessOrEqual( DirectX::XMVector3LengthSq( Center - Point ), RadiusSq ) );

    // Edge 1,2
    Point = PointOnLineSegmentNearestPoint( V1, V2, Center );

    // If the distance to the center of the sphere to the point is less than 
    // the radius of the sphere then it must intersect.
    Intersection = DirectX::XMVectorOrInt( Intersection, DirectX::XMVectorLessOrEqual( DirectX::XMVector3LengthSq( Center - Point ), RadiusSq ) );

    // Edge 2,0
    Point = PointOnLineSegmentNearestPoint( V2, V0, Center );

    // If the distance to the center of the sphere to the point is less than 
    // the radius of the sphere then it must intersect.
    Intersection = DirectX::XMVectorOrInt( Intersection, DirectX::XMVectorLessOrEqual( DirectX::XMVector3LengthSq( Center - Point ), RadiusSq ) );

    return DirectX::XMVector4EqualInt( DirectX::XMVectorAndCInt( Intersection, NoIntersection ), DirectX::XMVectorTrueInt() );
}



//-----------------------------------------------------------------------------
BOOL IntersectTriangleAxisAlignedBox( FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::FXMVECTOR V2, const AxisAlignedBox* pVolume )
{
    XMASSERT( pVolume );

    static CONST XMVECTORI32 Permute0W1Z0Y0X =
                 {
                    DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_1Z, DirectX::XM_PERMUTE_0Y, DirectX::XM_PERMUTE_0X
                 };
    static CONST XMVECTORI32 Permute0Z0W1X0Y =
                 {
                    DirectX::XM_PERMUTE_0Z, DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_1X, DirectX::XM_PERMUTE_0Y
                 };
    static CONST XMVECTORI32 Permute1Y0X0W0Z =
                 {
                    DirectX::XM_PERMUTE_1Y, DirectX::XM_PERMUTE_0X, DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_0Z
                 };

    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );

    DirectX::XMVECTOR BoxMin = Center - Extents;
    DirectX::XMVECTOR BoxMax = Center + Extents;

    // Test the axes of the box (in effect test the AAB against the minimal AAB 
    // around the triangle).
    DirectX::XMVECTOR TriMin = DirectX::XMVectorMin( DirectX::XMVectorMin( V0, V1 ), V2 );
    DirectX::XMVECTOR TriMax = DirectX::XMVectorMax( DirectX::XMVectorMax( V0, V1 ), V2 );

    // for each i in (x, y, z) if a_min(i) > b_max(i) or b_min(i) > a_max(i) then disjoint
    DirectX::XMVECTOR Disjoint = DirectX::XMVectorOrInt( DirectX::XMVectorGreater( TriMin, BoxMax ), DirectX::XMVectorGreater( BoxMin, TriMax ) );
    if( DirectX::XMVector3AnyTrue( Disjoint ) )
        return FALSE;

    // Test the plane of the triangle.
    DirectX::XMVECTOR Normal = DirectX::XMVector3Cross( V1 - V0, V2 - V0 );
    DirectX::XMVECTOR Dist = DirectX::XMVector3Dot( Normal, V0 );

    // Assert that the triangle is not degenerate.
    XMASSERT( !DirectX::XMVector3Equal( Normal, Zero ) );

    // for each i in (x, y, z) if n(i) >= 0 then v_min(i)=b_min(i), v_max(i)=b_max(i)
    // else v_min(i)=b_max(i), v_max(i)=b_min(i)
    DirectX::XMVECTOR NormalSelect = DirectX::XMVectorGreater( Normal, Zero );
    DirectX::XMVECTOR V_Min = DirectX::XMVectorSelect( BoxMax, BoxMin, NormalSelect );
    DirectX::XMVECTOR V_Max = DirectX::XMVectorSelect( BoxMin, BoxMax, NormalSelect );

    // if n dot v_min + d > 0 || n dot v_max + d < 0 then disjoint
    DirectX::XMVECTOR MinDist = DirectX::XMVector3Dot( V_Min, Normal );
    DirectX::XMVECTOR MaxDist = DirectX::XMVector3Dot( V_Max, Normal );

    DirectX::XMVECTOR NoIntersection = DirectX::XMVectorGreater( MinDist, Dist );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( MaxDist, Dist ) );

    // Move the box center to zero to simplify the following tests.
    DirectX::XMVECTOR TV0 = V0 - Center;
    DirectX::XMVECTOR TV1 = V1 - Center;
    DirectX::XMVECTOR TV2 = V2 - Center;

    // Test the edge/edge axes (3*3).
    DirectX::XMVECTOR e0 = TV1 - TV0;
    DirectX::XMVECTOR e1 = TV2 - TV1;
    DirectX::XMVECTOR e2 = TV0 - TV2;

    // Make w zero.
    e0 = DirectX::XMVectorInsert( e0, Zero, 0, 0, 0, 0, 1 );
    e1 = DirectX::XMVectorInsert( e1, Zero, 0, 0, 0, 0, 1 );
    e2 = DirectX::XMVectorInsert( e2, Zero, 0, 0, 0, 0, 1 );

    DirectX::XMVECTOR Axis;
    DirectX::XMVECTOR p0, p1, p2;
    DirectX::XMVECTOR Min, Max;
    DirectX::XMVECTOR Radius;

    // Axis == (1,0,0) x e0 = (0, -e0.z, e0.y)
    Axis = DirectX::XMVectorPermute( e0, -e0, Permute0W1Z0Y0X );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    // p1 = DirectX::XMVector3Dot( V1, Axis ); // p1 = p0;
    p2 = DirectX::XMVector3Dot( TV2, Axis );
    Min = DirectX::XMVectorMin( p0, p2 );
    Max = DirectX::XMVectorMax( p0, p2 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    // Axis == (1,0,0) x e1 = (0, -e1.z, e1.y)
    Axis = DirectX::XMVectorPermute( e1, -e1, Permute0W1Z0Y0X );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    p1 = DirectX::XMVector3Dot( TV1, Axis );
    // p2 = DirectX::XMVector3Dot( V2, Axis ); // p2 = p1;
    Min = DirectX::XMVectorMin( p0, p1 );
    Max = DirectX::XMVectorMax( p0, p1 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    // Axis == (1,0,0) x e2 = (0, -e2.z, e2.y)
    Axis = DirectX::XMVectorPermute( e2, -e2, Permute0W1Z0Y0X );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    p1 = DirectX::XMVector3Dot( TV1, Axis );
    // p2 = DirectX::XMVector3Dot( V2, Axis ); // p2 = p0;
    Min = DirectX::XMVectorMin( p0, p1 );
    Max = DirectX::XMVectorMax( p0, p1 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    // Axis == (0,1,0) x e0 = (e0.z, 0, -e0.x)
    Axis = DirectX::XMVectorPermute( e0, -e0, Permute0Z0W1X0Y );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    // p1 = DirectX::XMVector3Dot( V1, Axis ); // p1 = p0;
    p2 = DirectX::XMVector3Dot( TV2, Axis );
    Min = DirectX::XMVectorMin( p0, p2 );
    Max = DirectX::XMVectorMax( p0, p2 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    // Axis == (0,1,0) x e1 = (e1.z, 0, -e1.x)
    Axis = DirectX::XMVectorPermute( e1, -e1, Permute0Z0W1X0Y );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    p1 = DirectX::XMVector3Dot( TV1, Axis );
    // p2 = DirectX::XMVector3Dot( V2, Axis ); // p2 = p1;
    Min = DirectX::XMVectorMin( p0, p1 );
    Max = DirectX::XMVectorMax( p0, p1 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    // Axis == (0,0,1) x e2 = (e2.z, 0, -e2.x)
    Axis = DirectX::XMVectorPermute( e2, -e2, Permute0Z0W1X0Y );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    p1 = DirectX::XMVector3Dot( TV1, Axis );
    // p2 = DirectX::XMVector3Dot( V2, Axis ); // p2 = p0;
    Min = DirectX::XMVectorMin( p0, p1 );
    Max = DirectX::XMVectorMax( p0, p1 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    // Axis == (0,0,1) x e0 = (-e0.y, e0.x, 0)
    Axis = DirectX::XMVectorPermute( e0, -e0, Permute1Y0X0W0Z );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    // p1 = DirectX::XMVector3Dot( V1, Axis ); // p1 = p0;
    p2 = DirectX::XMVector3Dot( TV2, Axis );
    Min = DirectX::XMVectorMin( p0, p2 );
    Max = DirectX::XMVectorMax( p0, p2 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    // Axis == (0,0,1) x e1 = (-e1.y, e1.x, 0)
    Axis = DirectX::XMVectorPermute( e1, -e1, Permute1Y0X0W0Z );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    p1 = DirectX::XMVector3Dot( TV1, Axis );
    // p2 = DirectX::XMVector3Dot( V2, Axis ); // p2 = p1;
    Min = DirectX::XMVectorMin( p0, p1 );
    Max = DirectX::XMVectorMax( p0, p1 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    // Axis == (0,0,1) x e2 = (-e2.y, e2.x, 0)
    Axis = DirectX::XMVectorPermute( e2, -e2, Permute1Y0X0W0Z );
    p0 = DirectX::XMVector3Dot( TV0, Axis );
    p1 = DirectX::XMVector3Dot( TV1, Axis );
    // p2 = DirectX::XMVector3Dot( V2, Axis ); // p2 = p0;
    Min = DirectX::XMVectorMin( p0, p1 );
    Max = DirectX::XMVectorMax( p0, p1 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Axis ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorGreater( Min, Radius ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, DirectX::XMVectorLess( Max, -Radius ) );

    return DirectX::XMVector4NotEqualInt( NoIntersection, DirectX::XMVectorTrueInt() );
}



//-----------------------------------------------------------------------------
BOOL IntersectTriangleOrientedBox( FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::FXMVECTOR V2, const OrientedBox* pVolume )
{
    XMASSERT( pVolume );

    // Load the box center & orientation.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Transform the triangle vertices into the space of the box.
    DirectX::XMVECTOR TV0 = DirectX::XMVector3InverseRotate( V0 - Center, Orientation );
    DirectX::XMVECTOR TV1 = DirectX::XMVector3InverseRotate( V1 - Center, Orientation );
    DirectX::XMVECTOR TV2 = DirectX::XMVector3InverseRotate( V2 - Center, Orientation );

    AxisAlignedBox Box;
    Box.Center = DirectX::XMFLOAT3( 0.0f, 0.0f, 0.0f );
    Box.Extents = pVolume->Extents;

    // Use the triangle vs axis aligned box intersection routine.
    return IntersectTriangleAxisAlignedBox( TV0, TV1, TV2, &Box );
}



//-----------------------------------------------------------------------------
BOOL IntersectSphereSphere( const Sphere* pVolumeA, const Sphere* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    // Load A.
    DirectX::XMVECTOR CenterA = XMLoadFloat3( &pVolumeA->Center );
    DirectX::XMVECTOR RadiusA = DirectX::XMVectorReplicatePtr( &pVolumeA->Radius );

    // Load B.
    DirectX::XMVECTOR CenterB = XMLoadFloat3( &pVolumeB->Center );
    DirectX::XMVECTOR RadiusB = DirectX::XMVectorReplicatePtr( &pVolumeB->Radius );

    // Distance squared between centers.    
    DirectX::XMVECTOR Delta = CenterB - CenterA;
    DirectX::XMVECTOR DistanceSquared = DirectX::XMVector3LengthSq( Delta );

    // Sum of the radii sqaured.
    DirectX::XMVECTOR RadiusSquared = RadiusA + RadiusB;
    RadiusSquared = RadiusSquared * RadiusSquared;

    return DirectX::XMVector4LessOrEqual( DistanceSquared, RadiusSquared );
}



//-----------------------------------------------------------------------------
BOOL IntersectSphereAxisAlignedBox( const Sphere* pVolumeA, const AxisAlignedBox* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    DirectX::XMVECTOR SphereCenter = XMLoadFloat3( &pVolumeA->Center );
    DirectX::XMVECTOR SphereRadius = DirectX::XMVectorReplicatePtr( &pVolumeA->Radius );

    DirectX::XMVECTOR BoxCenter = XMLoadFloat3( &pVolumeB->Center );
    DirectX::XMVECTOR BoxExtents = XMLoadFloat3( &pVolumeB->Extents );

    DirectX::XMVECTOR BoxMin = BoxCenter - BoxExtents;
    DirectX::XMVECTOR BoxMax = BoxCenter + BoxExtents;

    // Find the distance to the nearest point on the box.
    // for each i in (x, y, z)
    // if (SphereCenter(i) < BoxMin(i)) d2 += (SphereCenter(i) - BoxMin(i)) ^ 2
    // else if (SphereCenter(i) > BoxMax(i)) d2 += (SphereCenter(i) - BoxMax(i)) ^ 2

    DirectX::XMVECTOR d = DirectX::XMVectorZero();

    // Compute d for each dimension.
    DirectX::XMVECTOR LessThanMin = DirectX::XMVectorLess( SphereCenter, BoxMin );
    DirectX::XMVECTOR GreaterThanMax = DirectX::XMVectorGreater( SphereCenter, BoxMax );

    DirectX::XMVECTOR MinDelta = SphereCenter - BoxMin;
    DirectX::XMVECTOR MaxDelta = SphereCenter - BoxMax;

    // Choose value for each dimension based on the comparison.
    d = DirectX::XMVectorSelect( d, MinDelta, LessThanMin );
    d = DirectX::XMVectorSelect( d, MaxDelta, GreaterThanMax );

    // Use a dot-product to square them and sum them together.
    DirectX::XMVECTOR d2 = DirectX::XMVector3Dot( d, d );

    return DirectX::XMVector4LessOrEqual( d2, DirectX::XMVectorMultiply( SphereRadius, SphereRadius ) );
}



//-----------------------------------------------------------------------------
BOOL IntersectSphereOrientedBox( const Sphere* pVolumeA, const OrientedBox* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    DirectX::XMVECTOR SphereCenter = XMLoadFloat3( &pVolumeA->Center );
    DirectX::XMVECTOR SphereRadius = DirectX::XMVectorReplicatePtr( &pVolumeA->Radius );

    DirectX::XMVECTOR BoxCenter = XMLoadFloat3( &pVolumeB->Center );
    DirectX::XMVECTOR BoxExtents = XMLoadFloat3( &pVolumeB->Extents );
    DirectX::XMVECTOR BoxOrientation = XMLoadFloat4( &pVolumeB->Orientation );

    XMASSERT( XMQuaternionIsUnit( BoxOrientation ) );

    // Transform the center of the sphere to be local to the box.
    // BoxMin = -BoxExtents
    // BoxMax = +BoxExtents
    SphereCenter = DirectX::XMVector3InverseRotate( SphereCenter - BoxCenter, BoxOrientation );

    // Find the distance to the nearest point on the box.
    // for each i in (x, y, z)
    // if (SphereCenter(i) < BoxMin(i)) d2 += (SphereCenter(i) - BoxMin(i)) ^ 2
    // else if (SphereCenter(i) > BoxMax(i)) d2 += (SphereCenter(i) - BoxMax(i)) ^ 2

    DirectX::XMVECTOR d = DirectX::XMVectorZero();

    // Compute d for each dimension.
    DirectX::XMVECTOR LessThanMin = DirectX::XMVectorLess( SphereCenter, -BoxExtents );
    DirectX::XMVECTOR GreaterThanMax = DirectX::XMVectorGreater( SphereCenter, BoxExtents );

    DirectX::XMVECTOR MinDelta = SphereCenter + BoxExtents;
    DirectX::XMVECTOR MaxDelta = SphereCenter - BoxExtents;

    // Choose value for each dimension based on the comparison.
    d = DirectX::XMVectorSelect( d, MinDelta, LessThanMin );
    d = DirectX::XMVectorSelect( d, MaxDelta, GreaterThanMax );

    // Use a dot-product to square them and sum them together.
    DirectX::XMVECTOR d2 = DirectX::XMVector3Dot( d, d );

    return DirectX::XMVector4LessOrEqual( d2, DirectX::XMVectorMultiply( SphereRadius, SphereRadius ) );
}



//-----------------------------------------------------------------------------
BOOL IntersectAxisAlignedBoxAxisAlignedBox( const AxisAlignedBox* pVolumeA, const AxisAlignedBox* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    DirectX::XMVECTOR CenterA = XMLoadFloat3( &pVolumeA->Center );
    DirectX::XMVECTOR ExtentsA = XMLoadFloat3( &pVolumeA->Extents );

    DirectX::XMVECTOR CenterB = XMLoadFloat3( &pVolumeB->Center );
    DirectX::XMVECTOR ExtentsB = XMLoadFloat3( &pVolumeB->Extents );

    DirectX::XMVECTOR MinA = CenterA - ExtentsA;
    DirectX::XMVECTOR MaxA = CenterA + ExtentsA;

    DirectX::XMVECTOR MinB = CenterB - ExtentsB;
    DirectX::XMVECTOR MaxB = CenterB + ExtentsB;

    // for each i in (x, y, z) if a_min(i) > b_max(i) or b_min(i) > a_max(i) then return FALSE
    DirectX::XMVECTOR Disjoint = DirectX::XMVectorOrInt( DirectX::XMVectorGreater( MinA, MaxB ), DirectX::XMVectorGreater( MinB, MaxA ) );

    return !DirectX::XMVector3AnyTrue( Disjoint );
}



//-----------------------------------------------------------------------------
BOOL IntersectAxisAlignedBoxOrientedBox( const AxisAlignedBox* pVolumeA, const OrientedBox* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    // Make the axis aligned box oriented and do an OBB vs OBB test.
    OrientedBox BoxA;

    BoxA.Center = pVolumeA->Center;
    BoxA.Extents = pVolumeA->Extents;
    BoxA.Orientation.x = 0.0f;
    BoxA.Orientation.y = 0.0f;
    BoxA.Orientation.z = 0.0f;
    BoxA.Orientation.w = 1.0f;

    return IntersectOrientedBoxOrientedBox( &BoxA, pVolumeB );
}



//-----------------------------------------------------------------------------
// Fast oriented box / oriented box intersection test using the separating axis 
// theorem.
//-----------------------------------------------------------------------------
BOOL IntersectOrientedBoxOrientedBox( const OrientedBox* pVolumeA, const OrientedBox* pVolumeB )
{
    static CONST XMVECTORI32 Permute0W1Z0Y0X =
                 {
                    DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_1Z, DirectX::XM_PERMUTE_0Y, DirectX::XM_PERMUTE_0X
                 };
    static CONST XMVECTORI32 Permute0Z0W1X0Y =
                 {
                    DirectX::XM_PERMUTE_0Z, DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_1X, DirectX::XM_PERMUTE_0Y
                 };
    static CONST XMVECTORI32 Permute1Y0X0W0Z =
                 {
                    DirectX::XM_PERMUTE_1Y, DirectX::XM_PERMUTE_0X, DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_0Z
                 };
    static CONST XMVECTORI32 PermuteWZYX =
                 {
                    DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_0Z, DirectX::XM_PERMUTE_0Y, DirectX::XM_PERMUTE_0X
                 };
    static CONST XMVECTORI32 PermuteZWXY =
                 {
                    DirectX::XM_PERMUTE_0Z, DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_0X, DirectX::XM_PERMUTE_0Y
                 };
    static CONST XMVECTORI32 PermuteYXWZ =
                 {
                    DirectX::XM_PERMUTE_0Y, DirectX::XM_PERMUTE_0X, DirectX::XM_PERMUTE_0W, DirectX::XM_PERMUTE_0Z
                 };

    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    // Build the 3x3 rotation matrix that defines the orientation of B relative to A.
    DirectX::XMVECTOR A_quat = XMLoadFloat4( &pVolumeA->Orientation );
    DirectX::XMVECTOR B_quat = XMLoadFloat4( &pVolumeB->Orientation );

    XMASSERT( XMQuaternionIsUnit( A_quat ) );
    XMASSERT( XMQuaternionIsUnit( B_quat ) );

    DirectX::XMVECTOR Q = XMQuaternionMultiply( A_quat, XMQuaternionConjugate( B_quat ) );
    DirectX::XMMATRIX R = DirectX::XMMatrixRotationQuaternion( Q );

    // Compute the translation of B relative to A.
    DirectX::XMVECTOR A_cent = XMLoadFloat3( &pVolumeA->Center );
    DirectX::XMVECTOR B_cent = XMLoadFloat3( &pVolumeB->Center );
    DirectX::XMVECTOR t = DirectX::XMVector3InverseRotate( B_cent - A_cent, A_quat );

    //
    // h(A) = extents of A.
    // h(B) = extents of B.
    //
    // a(u) = axes of A = (1,0,0), (0,1,0), (0,0,1)
    // b(u) = axes of B relative to A = (r00,r10,r20), (r01,r11,r21), (r02,r12,r22)
    //  
    // For each possible separating axis l:
    //   d(A) = sum (for i = u,v,w) h(A)(i) * abs( a(i) dot l )
    //   d(B) = sum (for i = u,v,w) h(B)(i) * abs( b(i) dot l )
    //   if abs( t dot l ) > d(A) + d(B) then disjoint
    //

    // Load extents of A and B.
    DirectX::XMVECTOR h_A = XMLoadFloat3( &pVolumeA->Extents );
    DirectX::XMVECTOR h_B = XMLoadFloat3( &pVolumeB->Extents );

    // Rows. Note R[0,1,2]X.w = 0.
    DirectX::XMVECTOR R0X = R.r[0];
    DirectX::XMVECTOR R1X = R.r[1];
    DirectX::XMVECTOR R2X = R.r[2];

    R = DirectX::XMMatrixTranspose( R );

    // Columns. Note RX[0,1,2].w = 0.
    DirectX::XMVECTOR RX0 = R.r[0];
    DirectX::XMVECTOR RX1 = R.r[1];
    DirectX::XMVECTOR RX2 = R.r[2];

    // Absolute value of rows.
    DirectX::XMVECTOR AR0X = DirectX::XMVectorAbs( R0X );
    DirectX::XMVECTOR AR1X = DirectX::XMVectorAbs( R1X );
    DirectX::XMVECTOR AR2X = DirectX::XMVectorAbs( R2X );

    // Absolute value of columns.
    DirectX::XMVECTOR ARX0 = DirectX::XMVectorAbs( RX0 );
    DirectX::XMVECTOR ARX1 = DirectX::XMVectorAbs( RX1 );
    DirectX::XMVECTOR ARX2 = DirectX::XMVectorAbs( RX2 );

    // Test each of the 15 possible seperating axii.
    DirectX::XMVECTOR d, d_A, d_B;

    // l = a(u) = (1, 0, 0)
    // t dot l = t.x
    // d(A) = h(A).x
    // d(B) = h(B) dot abs(r00, r01, r02)
    d = DirectX::XMVectorSplatX( t );
    d_A = DirectX::XMVectorSplatX( h_A );
    d_B = DirectX::XMVector3Dot( h_B, AR0X );
    DirectX::XMVECTOR NoIntersection = DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) );

    // l = a(v) = (0, 1, 0)
    // t dot l = t.y
    // d(A) = h(A).y
    // d(B) = h(B) dot abs(r10, r11, r12)
    d = DirectX::XMVectorSplatY( t );
    d_A = DirectX::XMVectorSplatY( h_A );
    d_B = DirectX::XMVector3Dot( h_B, AR1X );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(w) = (0, 0, 1)
    // t dot l = t.z
    // d(A) = h(A).z
    // d(B) = h(B) dot abs(r20, r21, r22)
    d = DirectX::XMVectorSplatZ( t );
    d_A = DirectX::XMVectorSplatZ( h_A );
    d_B = DirectX::XMVector3Dot( h_B, AR2X );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = b(u) = (r00, r10, r20)
    // d(A) = h(A) dot abs(r00, r10, r20)
    // d(B) = h(B).x
    d = DirectX::XMVector3Dot( t, RX0 );
    d_A = DirectX::XMVector3Dot( h_A, ARX0 );
    d_B = DirectX::XMVectorSplatX( h_B );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = b(v) = (r01, r11, r21)
    // d(A) = h(A) dot abs(r01, r11, r21)
    // d(B) = h(B).y
    d = DirectX::XMVector3Dot( t, RX1 );
    d_A = DirectX::XMVector3Dot( h_A, ARX1 );
    d_B = DirectX::XMVectorSplatY( h_B );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = b(w) = (r02, r12, r22)
    // d(A) = h(A) dot abs(r02, r12, r22)
    // d(B) = h(B).z
    d = DirectX::XMVector3Dot( t, RX2 );
    d_A = DirectX::XMVector3Dot( h_A, ARX2 );
    d_B = DirectX::XMVectorSplatZ( h_B );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(u) x b(u) = (0, -r20, r10)
    // d(A) = h(A) dot abs(0, r20, r10)
    // d(B) = h(B) dot abs(0, r02, r01)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX0, -RX0, Permute0W1Z0Y0X ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX0, ARX0, PermuteWZYX ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR0X, AR0X, PermuteWZYX ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(u) x b(v) = (0, -r21, r11)
    // d(A) = h(A) dot abs(0, r21, r11)
    // d(B) = h(B) dot abs(r02, 0, r00)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX1, -RX1, Permute0W1Z0Y0X ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX1, ARX1, PermuteWZYX ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR0X, AR0X, PermuteZWXY ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(u) x b(w) = (0, -r22, r12)
    // d(A) = h(A) dot abs(0, r22, r12)
    // d(B) = h(B) dot abs(r01, r00, 0)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX2, -RX2, Permute0W1Z0Y0X ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX2, ARX2, PermuteWZYX ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR0X, AR0X, PermuteYXWZ ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(v) x b(u) = (r20, 0, -r00)
    // d(A) = h(A) dot abs(r20, 0, r00)
    // d(B) = h(B) dot abs(0, r12, r11)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX0, -RX0, Permute0Z0W1X0Y ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX0, ARX0, PermuteZWXY ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR1X, AR1X, PermuteWZYX ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(v) x b(v) = (r21, 0, -r01)
    // d(A) = h(A) dot abs(r21, 0, r01)
    // d(B) = h(B) dot abs(r12, 0, r10)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX1, -RX1, Permute0Z0W1X0Y ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX1, ARX1, PermuteZWXY ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR1X, AR1X, PermuteZWXY ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(v) x b(w) = (r22, 0, -r02)
    // d(A) = h(A) dot abs(r22, 0, r02)
    // d(B) = h(B) dot abs(r11, r10, 0)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX2, -RX2, Permute0Z0W1X0Y ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX2, ARX2, PermuteZWXY ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR1X, AR1X, PermuteYXWZ ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(w) x b(u) = (-r10, r00, 0)
    // d(A) = h(A) dot abs(r10, r00, 0)
    // d(B) = h(B) dot abs(0, r22, r21)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX0, -RX0, Permute1Y0X0W0Z ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX0, ARX0, PermuteYXWZ ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR2X, AR2X, PermuteWZYX ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(w) x b(v) = (-r11, r01, 0)
    // d(A) = h(A) dot abs(r11, r01, 0)
    // d(B) = h(B) dot abs(r22, 0, r20)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX1, -RX1, Permute1Y0X0W0Z ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX1, ARX1, PermuteYXWZ ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR2X, AR2X, PermuteZWXY ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // l = a(w) x b(w) = (-r12, r02, 0)
    // d(A) = h(A) dot abs(r12, r02, 0)
    // d(B) = h(B) dot abs(r21, r20, 0)
    d = DirectX::XMVector3Dot( t, DirectX::XMVectorPermute( RX2, -RX2, Permute1Y0X0W0Z ) );
    d_A = DirectX::XMVector3Dot( h_A, DirectX::XMVectorPermute( ARX2, ARX2, PermuteYXWZ ) );
    d_B = DirectX::XMVector3Dot( h_B, DirectX::XMVectorPermute( AR2X, AR2X, PermuteYXWZ ) );
    NoIntersection = DirectX::XMVectorOrInt( NoIntersection, 
                                    DirectX::XMVectorGreater( DirectX::XMVectorAbs(d), DirectX::XMVectorAdd( d_A, d_B ) ) );

    // No seperating axis found, boxes must intersect.
    return DirectX::XMVector4NotEqualInt( NoIntersection, DirectX::XMVectorTrueInt() );
}



//-----------------------------------------------------------------------------
// Exact triangle vs frustum test.
// Return values: 0 = no intersection, 
//                1 = intersection, 
//                2 = triangle is completely inside frustum
//-----------------------------------------------------------------------------
INT IntersectTriangleFrustum( FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::FXMVECTOR V2, const Frustum* pVolume )
{
    XMASSERT( pVolume );

    // Build the frustum planes (NOTE: D is negated from the usual).
    DirectX::XMVECTOR Planes[6];
    Planes[0] = DirectX::XMVectorSet( 0.0f, 0.0f, -1.0f, -pVolume->Near );
    Planes[1] = DirectX::XMVectorSet( 0.0f, 0.0f, 1.0f, pVolume->Far );
    Planes[2] = DirectX::XMVectorSet( 1.0f, 0.0f, -pVolume->RightSlope, 0.0f );
    Planes[3] = DirectX::XMVectorSet( -1.0f, 0.0f, pVolume->LeftSlope, 0.0f );
    Planes[4] = DirectX::XMVectorSet( 0.0f, 1.0f, -pVolume->TopSlope, 0.0f );
    Planes[5] = DirectX::XMVectorSet( 0.0f, -1.0f, pVolume->BottomSlope, 0.0f );

    // Load origin and orientation of the frustum.
    DirectX::XMVECTOR Origin = XMLoadFloat3( &pVolume->Origin );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Transform triangle into the local space of frustum.
    DirectX::XMVECTOR TV0 = DirectX::XMVector3InverseRotate( V0 - Origin, Orientation );
    DirectX::XMVECTOR TV1 = DirectX::XMVector3InverseRotate( V1 - Origin, Orientation );
    DirectX::XMVECTOR TV2 = DirectX::XMVector3InverseRotate( V2 - Origin, Orientation );

    // Test each vertex of the triangle against the frustum planes.
    DirectX::XMVECTOR Outside = DirectX::XMVectorFalseInt();
    DirectX::XMVECTOR InsideAll = DirectX::XMVectorTrueInt();

    for( INT i = 0; i < 6; i++ )
    {
        DirectX::XMVECTOR Dist0 = DirectX::XMVector3Dot( TV0, Planes[i] );
        DirectX::XMVECTOR Dist1 = DirectX::XMVector3Dot( TV1, Planes[i] );
        DirectX::XMVECTOR Dist2 = DirectX::XMVector3Dot( TV2, Planes[i] );

        DirectX::XMVECTOR MinDist = DirectX::XMVectorMin( Dist0, Dist1 );
        MinDist = DirectX::XMVectorMin( MinDist, Dist2 );
        DirectX::XMVECTOR MaxDist = DirectX::XMVectorMax( Dist0, Dist1 );
        MaxDist = DirectX::XMVectorMax( MaxDist, Dist2 );

        DirectX::XMVECTOR PlaneDist = DirectX::XMVectorSplatW( Planes[i] );

        // Outside the plane?
        Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( MinDist, PlaneDist ) );

        // Fully inside the plane?
        InsideAll = DirectX::XMVectorAndInt( InsideAll, DirectX::XMVectorLessOrEqual( MaxDist, PlaneDist ) );
    }

    // If the triangle is outside any of the planes it is outside. 
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the triangle is inside all planes it is fully inside.
    if ( DirectX::XMVector4EqualInt( InsideAll, DirectX::XMVectorTrueInt() ) )
        return 2;

    // Build the corners of the frustum.
    DirectX::XMVECTOR RightTop = DirectX::XMVectorSet( pVolume->RightSlope, pVolume->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR RightBottom = DirectX::XMVectorSet( pVolume->RightSlope, pVolume->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftTop = DirectX::XMVectorSet( pVolume->LeftSlope, pVolume->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftBottom = DirectX::XMVectorSet( pVolume->LeftSlope, pVolume->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR Near = DirectX::XMVectorReplicatePtr( &pVolume->Near );
    DirectX::XMVECTOR Far = DirectX::XMVectorReplicatePtr( &pVolume->Far );

    DirectX::XMVECTOR Corners[8];
    Corners[0] = RightTop * Near;
    Corners[1] = RightBottom * Near;
    Corners[2] = LeftTop * Near;
    Corners[3] = LeftBottom * Near;
    Corners[4] = RightTop * Far;
    Corners[5] = RightBottom * Far;
    Corners[6] = LeftTop * Far;
    Corners[7] = LeftBottom * Far;

    // Test the plane of the triangle.
    DirectX::XMVECTOR Normal = DirectX::XMVector3Cross( V1 - V0, V2 - V0 );
    DirectX::XMVECTOR Dist = DirectX::XMVector3Dot( Normal, V0 );

    DirectX::XMVECTOR MinDist, MaxDist;
    MinDist = MaxDist = DirectX::XMVector3Dot( Corners[0], Normal );
    for( INT i = 1; i < 8; i++ )
    {
        DirectX::XMVECTOR Temp = DirectX::XMVector3Dot( Corners[i], Normal );
        MinDist = DirectX::XMVectorMin( MinDist, Temp );
        MaxDist = DirectX::XMVectorMax( MaxDist, Temp );
    }

    Outside = DirectX::XMVectorOrInt( DirectX::XMVectorGreater( MinDist, Dist ), DirectX::XMVectorLess( MaxDist, Dist ) );   
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // Check the edge/edge axes (3*6).
    DirectX::XMVECTOR TriangleEdgeAxis[3];
    TriangleEdgeAxis[0] = V1 - V0;
    TriangleEdgeAxis[1] = V2 - V1;
    TriangleEdgeAxis[2] = V0 - V2;

    DirectX::XMVECTOR FrustumEdgeAxis[6];
    FrustumEdgeAxis[0] = RightTop;
    FrustumEdgeAxis[1] = RightBottom;
    FrustumEdgeAxis[2] = LeftTop;
    FrustumEdgeAxis[3] = LeftBottom;
    FrustumEdgeAxis[4] = RightTop - LeftTop;
    FrustumEdgeAxis[5] = LeftBottom - LeftTop;

    for( INT i = 0; i < 3; i++ )
    {
        for( INT j = 0; j < 6; j++ )
        {
            // Compute the axis we are going to test.
            DirectX::XMVECTOR Axis = DirectX::XMVector3Cross( TriangleEdgeAxis[i], FrustumEdgeAxis[j] );

            // Find the min/max of the projection of the triangle onto the axis.
            DirectX::XMVECTOR MinA, MaxA;

            DirectX::XMVECTOR Dist0 = DirectX::XMVector3Dot( V0, Axis );
            DirectX::XMVECTOR Dist1 = DirectX::XMVector3Dot( V1, Axis );
            DirectX::XMVECTOR Dist2 = DirectX::XMVector3Dot( V2, Axis );

            MinA = DirectX::XMVectorMin( Dist0, Dist1 );
            MinA = DirectX::XMVectorMin( MinA, Dist2 );
            MaxA = DirectX::XMVectorMax( Dist0, Dist1 );
            MaxA = DirectX::XMVectorMax( MaxA, Dist2 );

            // Find the min/max of the projection of the frustum onto the axis.
            DirectX::XMVECTOR MinB, MaxB;

            MinB = MaxB = DirectX::XMVector3Dot( Axis, Corners[0] );

            for( INT k = 1; k < 8; k++ )
            {
                DirectX::XMVECTOR Temp = DirectX::XMVector3Dot( Axis, Corners[k] );
                MinB = DirectX::XMVectorMin( MinB, Temp );
                MaxB = DirectX::XMVectorMax( MaxB, Temp );
            }

            // if (MinA > MaxB || MinB > MaxA) reject;
            Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( MinA, MaxB ) );
            Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( MinB, MaxA ) );
        }
    }

    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If we did not find a separating plane then the triangle must intersect the frustum.
    return 1;
}



//-----------------------------------------------------------------------------
// Exact sphere vs frustum test.  The algorithm first checks the sphere against
// the planes of the frustum, then if the plane checks were indeterminate finds
// the nearest feature (plane, line, point) on the frustum to the center of the
// sphere and compares the distance to the nearest feature to the radius of the 
// sphere (it is so cool that all the comment lines above are the same length).
// Return values: 0 = no intersection, 
//                1 = intersection, 
//                2 = sphere is completely inside frustum
//-----------------------------------------------------------------------------
INT IntersectSphereFrustum( const Sphere* pVolumeA, const Frustum* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();

    // Build the frustum planes.
    DirectX::XMVECTOR Planes[6];
    Planes[0] = DirectX::XMVectorSet( 0.0f, 0.0f, -1.0f, pVolumeB->Near );
    Planes[1] = DirectX::XMVectorSet( 0.0f, 0.0f, 1.0f, -pVolumeB->Far );
    Planes[2] = DirectX::XMVectorSet( 1.0f, 0.0f, -pVolumeB->RightSlope, 0.0f );
    Planes[3] = DirectX::XMVectorSet( -1.0f, 0.0f, pVolumeB->LeftSlope, 0.0f );
    Planes[4] = DirectX::XMVectorSet( 0.0f, 1.0f, -pVolumeB->TopSlope, 0.0f );
    Planes[5] = DirectX::XMVectorSet( 0.0f, -1.0f, pVolumeB->BottomSlope, 0.0f );

    // Normalize the planes so we can compare to the sphere radius.
    Planes[2] = DirectX::XMVector3Normalize( Planes[2] );
    Planes[3] = DirectX::XMVector3Normalize( Planes[3] );
    Planes[4] = DirectX::XMVector3Normalize( Planes[4] );
    Planes[5] = DirectX::XMVector3Normalize( Planes[5] );

    // Load origin and orientation of the frustum.
    DirectX::XMVECTOR Origin = XMLoadFloat3( &pVolumeB->Origin );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolumeB->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Load the sphere.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolumeA->Center );
    DirectX::XMVECTOR Radius = DirectX::XMVectorReplicatePtr( &pVolumeA->Radius );

    // Transform the center of the sphere into the local space of frustum.
    Center = DirectX::XMVector3InverseRotate( Center - Origin, Orientation );

    // Set w of the center to one so we can dot4 with the plane.
    Center = DirectX::XMVectorInsert( Center, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1);

    // Check against each plane of the frustum.
    DirectX::XMVECTOR Outside = DirectX::XMVectorFalseInt();
    DirectX::XMVECTOR InsideAll = DirectX::XMVectorTrueInt();
    DirectX::XMVECTOR CenterInsideAll = DirectX::XMVectorTrueInt();

    DirectX::XMVECTOR Dist[6];

    for( INT i = 0; i < 6; i++ )
    {
        Dist[i] = DirectX::XMVector4Dot( Center, Planes[i] );

        // Outside the plane?
        Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( Dist[i], Radius ) );

        // Fully inside the plane?
        InsideAll = DirectX::XMVectorAndInt( InsideAll, DirectX::XMVectorLessOrEqual( Dist[i], -Radius ) );

        // Check if the center is inside the plane.
        CenterInsideAll = DirectX::XMVectorAndInt( CenterInsideAll, DirectX::XMVectorLessOrEqual( Dist[i], Zero ) );
    }

    // If the sphere is outside any of the planes it is outside. 
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the sphere is inside all planes it is fully inside.
    if ( DirectX::XMVector4EqualInt( InsideAll, DirectX::XMVectorTrueInt() ) )
        return 2;

    // If the center of the sphere is inside all planes and the sphere intersects 
    // one or more planes then it must intersect.
    if ( DirectX::XMVector4EqualInt( CenterInsideAll, DirectX::XMVectorTrueInt() ) )
        return 1;

    // The sphere may be outside the frustum or intersecting the frustum.
    // Find the nearest feature (face, edge, or corner) on the frustum 
    // to the sphere.

    // The faces adjacent to each face are:
    static const INT adjacent_faces[6][4] =
    {
        { 2, 3, 4, 5 },    // 0
        { 2, 3, 4, 5 },    // 1
        { 0, 1, 4, 5 },    // 2
        { 0, 1, 4, 5 },    // 3
        { 0, 1, 2, 3 },    // 4
        { 0, 1, 2, 3 }
    };  // 5

    DirectX::XMVECTOR Intersects = DirectX::XMVectorFalseInt();

    // Check to see if the nearest feature is one of the planes.
    for( INT i = 0; i < 6; i++ )
    {
        // Find the nearest point on the plane to the center of the sphere.
        DirectX::XMVECTOR Point = Center - (Planes[i] * Dist[i]);

        // Set w of the point to one.
        Point = DirectX::XMVectorInsert( Point, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1 );
        
        // If the point is inside the face (inside the adjacent planes) then
        // this plane is the nearest feature.
        DirectX::XMVECTOR InsideFace = DirectX::XMVectorTrueInt();
        
        for ( INT j = 0; j < 4; j++ )
        {
            INT plane_index = adjacent_faces[i][j];

            InsideFace = DirectX::XMVectorAndInt( InsideFace,
                           DirectX::XMVectorLessOrEqual( DirectX::XMVector4Dot( Point, Planes[plane_index] ), Zero ) );
        }
     
        // Since we have already checked distance from the plane we know that the
        // sphere must intersect if this plane is the nearest feature.
        Intersects = DirectX::XMVectorOrInt( Intersects, 
                                    DirectX::XMVectorAndInt( DirectX::XMVectorGreater( Dist[i], Zero ), InsideFace ) );
    }

    if ( DirectX::XMVector4EqualInt( Intersects, DirectX::XMVectorTrueInt() ) )
        return 1;

    // Build the corners of the frustum.
    DirectX::XMVECTOR RightTop = DirectX::XMVectorSet( pVolumeB->RightSlope, pVolumeB->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR RightBottom = DirectX::XMVectorSet( pVolumeB->RightSlope, pVolumeB->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftTop = DirectX::XMVectorSet( pVolumeB->LeftSlope, pVolumeB->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftBottom = DirectX::XMVectorSet( pVolumeB->LeftSlope, pVolumeB->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR Near = DirectX::XMVectorReplicatePtr( &pVolumeB->Near );
    DirectX::XMVECTOR Far = DirectX::XMVectorReplicatePtr( &pVolumeB->Far );

    DirectX::XMVECTOR Corners[8];
    Corners[0] = RightTop * Near;
    Corners[1] = RightBottom * Near;
    Corners[2] = LeftTop * Near;
    Corners[3] = LeftBottom * Near;
    Corners[4] = RightTop * Far;
    Corners[5] = RightBottom * Far;
    Corners[6] = LeftTop * Far;
    Corners[7] = LeftBottom * Far;

    // The Edges are:
    static const INT edges[12][2] =
    {
        { 0, 1 }, { 2, 3 }, { 0, 2 }, { 1, 3 },    // Near plane
        { 4, 5 }, { 6, 7 }, { 4, 6 }, { 5, 7 },    // Far plane
        { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
    }; // Near to far

    DirectX::XMVECTOR RadiusSq = Radius * Radius;

    // Check to see if the nearest feature is one of the edges (or corners).
    for( INT i = 0; i < 12; i++ )
    {
        INT ei0 = edges[i][0];
        INT ei1 = edges[i][1];

        // Find the nearest point on the edge to the center of the sphere.
        // The corners of the frustum are included as the endpoints of the edges.
        DirectX::XMVECTOR Point = PointOnLineSegmentNearestPoint( Corners[ei0], Corners[ei1], Center );

        DirectX::XMVECTOR Delta = Center - Point;

        DirectX::XMVECTOR DistSq = DirectX::XMVector3Dot( Delta, Delta );

        // If the distance to the center of the sphere to the point is less than 
        // the radius of the sphere then it must intersect.
        Intersects = DirectX::XMVectorOrInt( Intersects, DirectX::XMVectorLessOrEqual( DistSq, RadiusSq ) );
    }

    if ( DirectX::XMVector4EqualInt( Intersects, DirectX::XMVectorTrueInt() ) )
        return 1;

    // The sphere must be outside the frustum.
    return 0;
}



//-----------------------------------------------------------------------------
// Exact axis alinged box vs frustum test.  Constructs an oriented box and uses
// the oriented box vs frustum test.
//
// Return values: 0 = no intersection, 
//                1 = intersection, 
//                2 = box is completely inside frustum
//-----------------------------------------------------------------------------
INT IntersectAxisAlignedBoxFrustum( const AxisAlignedBox* pVolumeA, const Frustum* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    // Make the axis aligned box oriented and do an OBB vs frustum test.
    OrientedBox BoxA;

    BoxA.Center = pVolumeA->Center;
    BoxA.Extents = pVolumeA->Extents;
    BoxA.Orientation.x = 0.0f;
    BoxA.Orientation.y = 0.0f;
    BoxA.Orientation.z = 0.0f;
    BoxA.Orientation.w = 1.0f;

    return IntersectOrientedBoxFrustum( &BoxA, pVolumeB );
}


//-----------------------------------------------------------------------------
// Exact oriented box vs frustum test.
// Return values: 0 = no intersection, 
//                1 = intersection, 
//                2 = box is completely inside frustum
//-----------------------------------------------------------------------------
INT IntersectOrientedBoxFrustum( const OrientedBox* pVolumeA, const Frustum* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    static const XMVECTORI32 SelectY =
    {
        DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_0, DirectX::XM_SELECT_0
    };
    static const XMVECTORI32 SelectZ =
    {
        DirectX::XM_SELECT_0, DirectX::XM_SELECT_0, DirectX::XM_SELECT_1, DirectX::XM_SELECT_0
    };

    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();

    // Build the frustum planes.
    DirectX::XMVECTOR Planes[6];
    Planes[0] = DirectX::XMVectorSet( 0.0f, 0.0f, -1.0f, pVolumeB->Near );
    Planes[1] = DirectX::XMVectorSet( 0.0f, 0.0f, 1.0f, -pVolumeB->Far );
    Planes[2] = DirectX::XMVectorSet( 1.0f, 0.0f, -pVolumeB->RightSlope, 0.0f );
    Planes[3] = DirectX::XMVectorSet( -1.0f, 0.0f, pVolumeB->LeftSlope, 0.0f );
    Planes[4] = DirectX::XMVectorSet( 0.0f, 1.0f, -pVolumeB->TopSlope, 0.0f );
    Planes[5] = DirectX::XMVectorSet( 0.0f, -1.0f, pVolumeB->BottomSlope, 0.0f );

    // Load origin and orientation of the frustum.
    DirectX::XMVECTOR Origin = XMLoadFloat3( &pVolumeB->Origin );
    DirectX::XMVECTOR FrustumOrientation = XMLoadFloat4( &pVolumeB->Orientation );

    XMASSERT( XMQuaternionIsUnit( FrustumOrientation ) );

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolumeA->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolumeA->Extents );
    DirectX::XMVECTOR BoxOrientation = XMLoadFloat4( &pVolumeA->Orientation );

    XMASSERT( XMQuaternionIsUnit( BoxOrientation ) );

    // Transform the oriented box into the space of the frustum in order to 
    // minimize the number of transforms we have to do.
    Center = DirectX::XMVector3InverseRotate( Center - Origin, FrustumOrientation );
    BoxOrientation = XMQuaternionMultiply( BoxOrientation, XMQuaternionConjugate( FrustumOrientation ) );

    // Set w of the center to one so we can dot4 with the plane.
    Center = DirectX::XMVectorInsert( Center, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1);

    // Build the 3x3 rotation matrix that defines the box axes.
    DirectX::XMMATRIX R = DirectX::XMMatrixRotationQuaternion( BoxOrientation );

    // Check against each plane of the frustum.
    DirectX::XMVECTOR Outside = DirectX::XMVectorFalseInt();
    DirectX::XMVECTOR InsideAll = DirectX::XMVectorTrueInt();
    DirectX::XMVECTOR CenterInsideAll = DirectX::XMVectorTrueInt();

    for( INT i = 0; i < 6; i++ )
    {
        // Compute the distance to the center of the box.
        DirectX::XMVECTOR Dist = DirectX::XMVector4Dot( Center, Planes[i] );

        // Project the axes of the box onto the normal of the plane.  Half the
        // length of the projection (sometime called the "radius") is equal to
        // h(u) * abs(n dot b(u))) + h(v) * abs(n dot b(v)) + h(w) * abs(n dot b(w))
        // where h(i) are extents of the box, n is the plane normal, and b(i) are the 
        // axes of the box.
        DirectX::XMVECTOR Radius = DirectX::XMVector3Dot( Planes[i], R.r[0] );
        Radius = DirectX::XMVectorSelect( Radius, DirectX::XMVector3Dot( Planes[i], R.r[1] ), SelectY );
        Radius = DirectX::XMVectorSelect( Radius, DirectX::XMVector3Dot( Planes[i], R.r[2] ), SelectZ );
        Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Radius ) );

        // Outside the plane?
        Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( Dist, Radius ) );

        // Fully inside the plane?
        InsideAll = DirectX::XMVectorAndInt( InsideAll, DirectX::XMVectorLessOrEqual( Dist, -Radius ) );

        // Check if the center is inside the plane.
        CenterInsideAll = DirectX::XMVectorAndInt( CenterInsideAll, DirectX::XMVectorLessOrEqual( Dist, Zero ) );
    }

    // If the box is outside any of the planes it is outside. 
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the box is inside all planes it is fully inside.
    if ( DirectX::XMVector4EqualInt( InsideAll, DirectX::XMVectorTrueInt() ) )
        return 2;

    // If the center of the box is inside all planes and the box intersects 
    // one or more planes then it must intersect.
    if ( DirectX::XMVector4EqualInt( CenterInsideAll, DirectX::XMVectorTrueInt() ) )
        return 1;

    // Build the corners of the frustum.
    DirectX::XMVECTOR RightTop = DirectX::XMVectorSet( pVolumeB->RightSlope, pVolumeB->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR RightBottom = DirectX::XMVectorSet( pVolumeB->RightSlope, pVolumeB->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftTop = DirectX::XMVectorSet( pVolumeB->LeftSlope, pVolumeB->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftBottom = DirectX::XMVectorSet( pVolumeB->LeftSlope, pVolumeB->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR Near = DirectX::XMVectorReplicatePtr( &pVolumeB->Near );
    DirectX::XMVECTOR Far = DirectX::XMVectorReplicatePtr( &pVolumeB->Far );

    DirectX::XMVECTOR Corners[8];
    Corners[0] = RightTop * Near;
    Corners[1] = RightBottom * Near;
    Corners[2] = LeftTop * Near;
    Corners[3] = LeftBottom * Near;
    Corners[4] = RightTop * Far;
    Corners[5] = RightBottom * Far;
    Corners[6] = LeftTop * Far;
    Corners[7] = LeftBottom * Far;

    // Test against box axes (3)
    {
        // Find the min/max values of the projection of the frustum onto each axis.
        DirectX::XMVECTOR FrustumMin, FrustumMax;

        FrustumMin = DirectX::XMVector3Dot( Corners[0], R.r[0] );
        FrustumMin = DirectX::XMVectorSelect( FrustumMin, DirectX::XMVector3Dot( Corners[0], R.r[1] ), SelectY );
        FrustumMin = DirectX::XMVectorSelect( FrustumMin, DirectX::XMVector3Dot( Corners[0], R.r[2] ), SelectZ );
        FrustumMax = FrustumMin;

        for( INT i = 1; i < 8; i++ )
        {
            DirectX::XMVECTOR Temp = DirectX::XMVector3Dot( Corners[i], R.r[0] );
            Temp = DirectX::XMVectorSelect( Temp, DirectX::XMVector3Dot( Corners[i], R.r[1] ), SelectY );
            Temp = DirectX::XMVectorSelect( Temp, DirectX::XMVector3Dot( Corners[i], R.r[2] ), SelectZ );

            FrustumMin = DirectX::XMVectorMin( FrustumMin, Temp );
            FrustumMax = DirectX::XMVectorMax( FrustumMax, Temp );
        }

        // Project the center of the box onto the axes.
        DirectX::XMVECTOR BoxDist = DirectX::XMVector3Dot( Center, R.r[0] );
        BoxDist = DirectX::XMVectorSelect( BoxDist, DirectX::XMVector3Dot( Center, R.r[1] ), SelectY );
        BoxDist = DirectX::XMVectorSelect( BoxDist, DirectX::XMVector3Dot( Center, R.r[2] ), SelectZ );

        // The projection of the box onto the axis is just its Center and Extents.
        // if (min > box_max || max < box_min) reject;
        DirectX::XMVECTOR Result = DirectX::XMVectorOrInt( DirectX::XMVectorGreater( FrustumMin, BoxDist + Extents ),
                                          DirectX::XMVectorLess( FrustumMax, BoxDist - Extents ) );

        if( DirectX::XMVector3AnyTrue( Result ) )
            return 0;
    }

    // Test against edge/edge axes (3*6).
    DirectX::XMVECTOR FrustumEdgeAxis[6];

    FrustumEdgeAxis[0] = RightTop;
    FrustumEdgeAxis[1] = RightBottom;
    FrustumEdgeAxis[2] = LeftTop;
    FrustumEdgeAxis[3] = LeftBottom;
    FrustumEdgeAxis[4] = RightTop - LeftTop;
    FrustumEdgeAxis[5] = LeftBottom - LeftTop;

    for( INT i = 0; i < 3; i++ )
    {
        for( INT j = 0; j < 6; j++ )
        {
            // Compute the axis we are going to test.
            DirectX::XMVECTOR Axis = DirectX::XMVector3Cross( R.r[i], FrustumEdgeAxis[j] );

            // Find the min/max values of the projection of the frustum onto the axis.
            DirectX::XMVECTOR FrustumMin, FrustumMax;

            FrustumMin = FrustumMax = DirectX::XMVector3Dot( Axis, Corners[0] );

            for( INT k = 1; k < 8; k++ )
            {
                DirectX::XMVECTOR Temp = DirectX::XMVector3Dot( Axis, Corners[k] );
                FrustumMin = DirectX::XMVectorMin( FrustumMin, Temp );
                FrustumMax = DirectX::XMVectorMax( FrustumMax, Temp );
            }

            // Project the center of the box onto the axis.
            DirectX::XMVECTOR Dist = DirectX::XMVector3Dot( Center, Axis );

            // Project the axes of the box onto the axis to find the "radius" of the box.
            DirectX::XMVECTOR Radius = DirectX::XMVector3Dot( Axis, R.r[0] );
            Radius = DirectX::XMVectorSelect( Radius, DirectX::XMVector3Dot( Axis, R.r[1] ), SelectY );
            Radius = DirectX::XMVectorSelect( Radius, DirectX::XMVector3Dot( Axis, R.r[2] ), SelectZ );
            Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Radius ) );

            // if (center > max + radius || center < min - radius) reject;
            Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( Dist, FrustumMax + Radius ) );
            Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorLess( Dist, FrustumMin - Radius ) );
        }
    }

    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If we did not find a separating plane then the box must intersect the frustum.
    return 1;
}



//-----------------------------------------------------------------------------
// Exact frustum vs frustum test.
// Return values: 0 = no intersection, 
//                1 = intersection, 
//                2 = frustum A is completely inside frustum B
//-----------------------------------------------------------------------------
INT IntersectFrustumFrustum( const Frustum* pVolumeA, const Frustum* pVolumeB )
{
    XMASSERT( pVolumeA );
    XMASSERT( pVolumeB );

    // Load origin and orientation of frustum B.
    DirectX::XMVECTOR OriginB = XMLoadFloat3( &pVolumeB->Origin );
    DirectX::XMVECTOR OrientationB = XMLoadFloat4( &pVolumeB->Orientation );

    XMASSERT( XMQuaternionIsUnit( OrientationB ) );

    // Build the planes of frustum B.
    DirectX::XMVECTOR AxisB[6];
    AxisB[0] = DirectX::XMVectorSet( 0.0f, 0.0f, -1.0f, 0.0f );
    AxisB[1] = DirectX::XMVectorSet( 0.0f, 0.0f, 1.0f, 0.0f );
    AxisB[2] = DirectX::XMVectorSet( 1.0f, 0.0f, -pVolumeB->RightSlope, 0.0f );
    AxisB[3] = DirectX::XMVectorSet( -1.0f, 0.0f, pVolumeB->LeftSlope, 0.0f );
    AxisB[4] = DirectX::XMVectorSet( 0.0f, 1.0f, -pVolumeB->TopSlope, 0.0f );
    AxisB[5] = DirectX::XMVectorSet( 0.0f, -1.0f, pVolumeB->BottomSlope, 0.0f );

    DirectX::XMVECTOR PlaneDistB[6];
    PlaneDistB[0] = -DirectX::XMVectorReplicatePtr( &pVolumeB->Near );
    PlaneDistB[1] = DirectX::XMVectorReplicatePtr( &pVolumeB->Far );
    PlaneDistB[2] = DirectX::XMVectorZero();
    PlaneDistB[3] = DirectX::XMVectorZero();
    PlaneDistB[4] = DirectX::XMVectorZero();
    PlaneDistB[5] = DirectX::XMVectorZero();

    // Load origin and orientation of frustum A.
    DirectX::XMVECTOR OriginA = XMLoadFloat3( &pVolumeA->Origin );
    DirectX::XMVECTOR OrientationA = XMLoadFloat4( &pVolumeA->Orientation );

    XMASSERT( XMQuaternionIsUnit( OrientationA ) );

    // Transform frustum A into the space of the frustum B in order to 
    // minimize the number of transforms we have to do.
    OriginA = DirectX::XMVector3InverseRotate( OriginA - OriginB, OrientationB );
    OrientationA = XMQuaternionMultiply( OrientationA, XMQuaternionConjugate( OrientationB ) );

    // Build the corners of frustum A (in the local space of B).
    DirectX::XMVECTOR RightTopA = DirectX::XMVectorSet( pVolumeA->RightSlope, pVolumeA->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR RightBottomA = DirectX::XMVectorSet( pVolumeA->RightSlope, pVolumeA->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftTopA = DirectX::XMVectorSet( pVolumeA->LeftSlope, pVolumeA->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftBottomA = DirectX::XMVectorSet( pVolumeA->LeftSlope, pVolumeA->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR NearA = DirectX::XMVectorReplicatePtr( &pVolumeA->Near );
    DirectX::XMVECTOR FarA = DirectX::XMVectorReplicatePtr( &pVolumeA->Far );

    RightTopA = DirectX::XMVector3Rotate( RightTopA, OrientationA );
    RightBottomA = DirectX::XMVector3Rotate( RightBottomA, OrientationA );
    LeftTopA = DirectX::XMVector3Rotate( LeftTopA, OrientationA );
    LeftBottomA = DirectX::XMVector3Rotate( LeftBottomA, OrientationA );

    DirectX::XMVECTOR CornersA[8];
    CornersA[0] = OriginA + RightTopA * NearA;
    CornersA[1] = OriginA + RightBottomA * NearA;
    CornersA[2] = OriginA + LeftTopA * NearA;
    CornersA[3] = OriginA + LeftBottomA * NearA;
    CornersA[4] = OriginA + RightTopA * FarA;
    CornersA[5] = OriginA + RightBottomA * FarA;
    CornersA[6] = OriginA + LeftTopA * FarA;
    CornersA[7] = OriginA + LeftBottomA * FarA;

    // Check frustum A against each plane of frustum B.
    DirectX::XMVECTOR Outside = DirectX::XMVectorFalseInt();
    DirectX::XMVECTOR InsideAll = DirectX::XMVectorTrueInt();

    for( INT i = 0; i < 6; i++ )
    {
        // Find the min/max projection of the frustum onto the plane normal.
        DirectX::XMVECTOR Min, Max;

        Min = Max = DirectX::XMVector3Dot( AxisB[i], CornersA[0] );

        for( INT j = 1; j < 8; j++ )
        {
            DirectX::XMVECTOR Temp = DirectX::XMVector3Dot( AxisB[i], CornersA[j] );
            Min = DirectX::XMVectorMin( Min, Temp );
            Max = DirectX::XMVectorMax( Max, Temp );
        }

        // Outside the plane?
        Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( Min, PlaneDistB[i] ) );

        // Fully inside the plane?
        InsideAll = DirectX::XMVectorAndInt( InsideAll, DirectX::XMVectorLessOrEqual( Max, PlaneDistB[i] ) );
    }

    // If the frustum A is outside any of the planes of frustum B it is outside. 
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If frustum A is inside all planes of frustum B it is fully inside.
    if ( DirectX::XMVector4EqualInt( InsideAll, DirectX::XMVectorTrueInt() ) )
        return 2;

    // Build the corners of frustum B.
    DirectX::XMVECTOR RightTopB = DirectX::XMVectorSet( pVolumeB->RightSlope, pVolumeB->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR RightBottomB = DirectX::XMVectorSet( pVolumeB->RightSlope, pVolumeB->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftTopB = DirectX::XMVectorSet( pVolumeB->LeftSlope, pVolumeB->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftBottomB = DirectX::XMVectorSet( pVolumeB->LeftSlope, pVolumeB->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR NearB = DirectX::XMVectorReplicatePtr( &pVolumeB->Near );
    DirectX::XMVECTOR FarB = DirectX::XMVectorReplicatePtr( &pVolumeB->Far );

    DirectX::XMVECTOR CornersB[8];
    CornersB[0] = RightTopB * NearB;
    CornersB[1] = RightBottomB * NearB;
    CornersB[2] = LeftTopB * NearB;
    CornersB[3] = LeftBottomB * NearB;
    CornersB[4] = RightTopB * FarB;
    CornersB[5] = RightBottomB * FarB;
    CornersB[6] = LeftTopB * FarB;
    CornersB[7] = LeftBottomB * FarB;

    // Build the planes of frustum A (in the local space of B).
    DirectX::XMVECTOR AxisA[6];
    DirectX::XMVECTOR PlaneDistA[6];

    AxisA[0] = DirectX::XMVectorSet( 0.0f, 0.0f, -1.0f, 0.0f );
    AxisA[1] = DirectX::XMVectorSet( 0.0f, 0.0f, 1.0f, 0.0f );
    AxisA[2] = DirectX::XMVectorSet( 1.0f, 0.0f, -pVolumeA->RightSlope, 0.0f );
    AxisA[3] = DirectX::XMVectorSet( -1.0f, 0.0f, pVolumeA->LeftSlope, 0.0f );
    AxisA[4] = DirectX::XMVectorSet( 0.0f, 1.0f, -pVolumeA->TopSlope, 0.0f );
    AxisA[5] = DirectX::XMVectorSet( 0.0f, -1.0f, pVolumeA->BottomSlope, 0.0f );

    AxisA[0] = DirectX::XMVector3Rotate( AxisA[0], OrientationA );
    AxisA[1] = -AxisA[0];
    AxisA[2] = DirectX::XMVector3Rotate( AxisA[2], OrientationA );
    AxisA[3] = DirectX::XMVector3Rotate( AxisA[3], OrientationA );
    AxisA[4] = DirectX::XMVector3Rotate( AxisA[4], OrientationA );
    AxisA[5] = DirectX::XMVector3Rotate( AxisA[5], OrientationA );

    PlaneDistA[0] = DirectX::XMVector3Dot( AxisA[0], CornersA[0] );  // Re-use corner on near plane.
    PlaneDistA[1] = DirectX::XMVector3Dot( AxisA[1], CornersA[4] );  // Re-use corner on far plane.
    PlaneDistA[2] = DirectX::XMVector3Dot( AxisA[2], OriginA );
    PlaneDistA[3] = DirectX::XMVector3Dot( AxisA[3], OriginA );
    PlaneDistA[4] = DirectX::XMVector3Dot( AxisA[4], OriginA );
    PlaneDistA[5] = DirectX::XMVector3Dot( AxisA[5], OriginA );

    // Check each axis of frustum A for a seperating plane (5).
    for( INT i = 0; i < 6; i++ )
    {
        // Find the minimum projection of the frustum onto the plane normal.
        DirectX::XMVECTOR Min;

        Min = DirectX::XMVector3Dot( AxisA[i], CornersB[0] );

        for( INT j = 1; j < 8; j++ )
        {
            DirectX::XMVECTOR Temp = DirectX::XMVector3Dot( AxisA[i], CornersB[j] );
            Min = DirectX::XMVectorMin( Min, Temp );
        }

        // Outside the plane?
        Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( Min, PlaneDistA[i] ) );
    }

    // If the frustum B is outside any of the planes of frustum A it is outside. 
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // Check edge/edge axes (6 * 6).
    DirectX::XMVECTOR FrustumEdgeAxisA[6];
    FrustumEdgeAxisA[0] = RightTopA;
    FrustumEdgeAxisA[1] = RightBottomA;
    FrustumEdgeAxisA[2] = LeftTopA;
    FrustumEdgeAxisA[3] = LeftBottomA;
    FrustumEdgeAxisA[4] = RightTopA - LeftTopA;
    FrustumEdgeAxisA[5] = LeftBottomA - LeftTopA;

    DirectX::XMVECTOR FrustumEdgeAxisB[6];
    FrustumEdgeAxisB[0] = RightTopB;
    FrustumEdgeAxisB[1] = RightBottomB;
    FrustumEdgeAxisB[2] = LeftTopB;
    FrustumEdgeAxisB[3] = LeftBottomB;
    FrustumEdgeAxisB[4] = RightTopB - LeftTopB;
    FrustumEdgeAxisB[5] = LeftBottomB - LeftTopB;

    for( INT i = 0; i < 6; i++ )
    {
        for( INT j = 0; j < 6; j++ )
        {
            // Compute the axis we are going to test.
            DirectX::XMVECTOR Axis = DirectX::XMVector3Cross( FrustumEdgeAxisA[i], FrustumEdgeAxisB[j] );

            // Find the min/max values of the projection of both frustums onto the axis.
            DirectX::XMVECTOR MinA, MaxA;
            DirectX::XMVECTOR MinB, MaxB;

            MinA = MaxA = DirectX::XMVector3Dot( Axis, CornersA[0] );
            MinB = MaxB = DirectX::XMVector3Dot( Axis, CornersB[0] );

            for( INT k = 1; k < 8; k++ )
            {
                DirectX::XMVECTOR TempA = DirectX::XMVector3Dot( Axis, CornersA[k] );
                MinA = DirectX::XMVectorMin( MinA, TempA );
                MaxA = DirectX::XMVectorMax( MaxA, TempA );

                DirectX::XMVECTOR TempB = DirectX::XMVector3Dot( Axis, CornersB[k] );
                MinB = DirectX::XMVectorMin( MinB, TempB );
                MaxB = DirectX::XMVectorMax( MaxB, TempB );
            }

            // if (MinA > MaxB || MinB > MaxA) reject
            Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( MinA, MaxB ) );
            Outside = DirectX::XMVectorOrInt( Outside, DirectX::XMVectorGreater( MinB, MaxA ) );
        }
    }

    // If there is a seperating plane, then the frustums do not intersect.
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If we did not find a separating plane then the frustums intersect.
    return 1;
}



//-----------------------------------------------------------------------------
static inline void FastIntersectTrianglePlane( FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::FXMVECTOR V2, CXMVECTOR Plane,
                                        DirectX::XMVECTOR& Outside, DirectX::XMVECTOR& Inside )
{
    // Plane0
    DirectX::XMVECTOR Dist0 = DirectX::XMVector4Dot( V0, Plane );
    DirectX::XMVECTOR Dist1 = DirectX::XMVector4Dot( V1, Plane );
    DirectX::XMVECTOR Dist2 = DirectX::XMVector4Dot( V2, Plane );

    DirectX::XMVECTOR MinDist = DirectX::XMVectorMin( Dist0, Dist1 );
    MinDist = DirectX::XMVectorMin( MinDist, Dist2 );

    DirectX::XMVECTOR MaxDist = DirectX::XMVectorMax( Dist0, Dist1 );
    MaxDist = DirectX::XMVectorMax( MaxDist, Dist2 );

    DirectX::XMVECTOR Zero = DirectX::XMVectorZero();

    // Outside the plane?
    Outside = DirectX::XMVectorGreater( MinDist, Zero );

    // Fully inside the plane?
    Inside = DirectX::XMVectorLess( MaxDist, Zero );
}



//-----------------------------------------------------------------------------
// Test a triangle vs 6 planes (typically forming a frustum).
// Return values: 0 = no intersection, 
//                1 = may be intersecting, 
//                2 = triangle is inside all planes
//-----------------------------------------------------------------------------
INT IntersectTriangle6Planes( FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::FXMVECTOR V2, CXMVECTOR Plane0, CXMVECTOR Plane1,
                              CXMVECTOR Plane2, CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 )
{
    DirectX::XMVECTOR One = DirectX::XMVectorSplatOne();

    // Set w of the points to one so we can dot4 with a plane.
    DirectX::XMVECTOR TV0 = DirectX::XMVectorInsert(V0, One, 0, 0, 0, 0, 1);
    DirectX::XMVECTOR TV1 = DirectX::XMVectorInsert(V1, One, 0, 0, 0, 0, 1);
    DirectX::XMVECTOR TV2 = DirectX::XMVectorInsert(V2, One, 0, 0, 0, 0, 1);

    DirectX::XMVECTOR Outside, Inside;

    // Test against each plane.
    FastIntersectTrianglePlane( TV0, TV1, TV2, Plane0, Outside, Inside );

    DirectX::XMVECTOR AnyOutside = Outside;
    DirectX::XMVECTOR AllInside = Inside;

    FastIntersectTrianglePlane( TV0, TV1, TV2, Plane1, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectTrianglePlane( TV0, TV1, TV2, Plane2, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectTrianglePlane( TV0, TV1, TV2, Plane3, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectTrianglePlane( TV0, TV1, TV2, Plane4, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectTrianglePlane( TV0, TV1, TV2, Plane5, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    // If the triangle is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( AnyOutside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the triangle is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( AllInside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The triangle is not inside all planes or outside a plane, it may intersect.
    return 1;
}



//-----------------------------------------------------------------------------
static inline void FastIntersectSpherePlane( FXMVECTOR Center, DirectX::FXMVECTOR Radius, DirectX::FXMVECTOR Plane,
                                      DirectX::XMVECTOR& Outside, DirectX::XMVECTOR& Inside )
{
    DirectX::XMVECTOR Dist = DirectX::XMVector4Dot( Center, Plane );

    // Outside the plane?
    Outside = DirectX::XMVectorGreater( Dist, Radius );

    // Fully inside the plane?
    Inside = DirectX::XMVectorLess( Dist, -Radius );
}



//-----------------------------------------------------------------------------
// Test a sphere vs 6 planes (typically forming a frustum).
// Return values: 0 = no intersection, 
//                1 = may be intersecting, 
//                2 = sphere is inside all planes
//-----------------------------------------------------------------------------
INT IntersectSphere6Planes( const Sphere* pVolume, DirectX::FXMVECTOR Plane0, DirectX::FXMVECTOR Plane1, DirectX::FXMVECTOR Plane2,
                            CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 )
{
    XMASSERT( pVolume );

    // Load the sphere.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Radius = DirectX::XMVectorReplicatePtr( &pVolume->Radius );

    // Set w of the center to one so we can dot4 with a plane.
    Center = DirectX::XMVectorInsert( Center, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1);

    DirectX::XMVECTOR Outside, Inside;

    // Test against each plane.
    FastIntersectSpherePlane( Center, Radius, Plane0, Outside, Inside );

    DirectX::XMVECTOR AnyOutside = Outside;
    DirectX::XMVECTOR AllInside = Inside;

    FastIntersectSpherePlane( Center, Radius, Plane1, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectSpherePlane( Center, Radius, Plane2, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectSpherePlane( Center, Radius, Plane3, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectSpherePlane( Center, Radius, Plane4, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectSpherePlane( Center, Radius, Plane5, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    // If the sphere is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( AnyOutside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the sphere is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( AllInside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The sphere is not inside all planes or outside a plane, it may intersect.
    return 1;
}



//-----------------------------------------------------------------------------
static inline void FastIntersectAxisAlignedBoxPlane( FXMVECTOR Center, DirectX::FXMVECTOR Extents, DirectX::FXMVECTOR Plane,
                                              DirectX::XMVECTOR& Outside, DirectX::XMVECTOR& Inside )
{
    // Compute the distance to the center of the box.
    DirectX::XMVECTOR Dist = DirectX::XMVector4Dot( Center, Plane );

    // Project the axes of the box onto the normal of the plane.  Half the
    // length of the projection (sometime called the "radius") is equal to
    // h(u) * abs(n dot b(u))) + h(v) * abs(n dot b(v)) + h(w) * abs(n dot b(w))
    // where h(i) are extents of the box, n is the plane normal, and b(i) are the 
    // axes of the box. In this case b(i) = [(1,0,0), (0,1,0), (0,0,1)].
    DirectX::XMVECTOR Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Plane ) );

    // Outside the plane?
    Outside = DirectX::XMVectorGreater( Dist, Radius );

    // Fully inside the plane?
    Inside = DirectX::XMVectorLess( Dist, -Radius );
}



//-----------------------------------------------------------------------------
// Test an axis alinged box vs 6 planes (typically forming a frustum).
// Return values: 0 = no intersection, 
//                1 = may be intersecting, 
//                2 = box is inside all planes
//-----------------------------------------------------------------------------
INT IntersectAxisAlignedBox6Planes( const AxisAlignedBox* pVolume, DirectX::FXMVECTOR Plane0, DirectX::FXMVECTOR Plane1,
                                    FXMVECTOR Plane2, CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 )
{
    XMASSERT( pVolume );

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );

    // Set w of the center to one so we can dot4 with a plane.
    Center = DirectX::XMVectorInsert( Center, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1 );

    DirectX::XMVECTOR Outside, Inside;

    // Test against each plane.
    FastIntersectAxisAlignedBoxPlane( Center, Extents, Plane0, Outside, Inside );

    DirectX::XMVECTOR AnyOutside = Outside;
    DirectX::XMVECTOR AllInside = Inside;

    FastIntersectAxisAlignedBoxPlane( Center, Extents, Plane1, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectAxisAlignedBoxPlane( Center, Extents, Plane2, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectAxisAlignedBoxPlane( Center, Extents, Plane3, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectAxisAlignedBoxPlane( Center, Extents, Plane4, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectAxisAlignedBoxPlane( Center, Extents, Plane5, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    // If the box is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( AnyOutside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the box is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( AllInside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The box is not inside all planes or outside a plane, it may intersect.
    return 1;
}



//-----------------------------------------------------------------------------
static inline void FastIntersectOrientedBoxPlane( FXMVECTOR Center, DirectX::FXMVECTOR Extents, DirectX::FXMVECTOR Axis0, CXMVECTOR Axis1,
                                          CXMVECTOR Axis2, CXMVECTOR Plane, DirectX::XMVECTOR& Outside, DirectX::XMVECTOR& Inside )
{
    // Compute the distance to the center of the box.
    DirectX::XMVECTOR Dist = DirectX::XMVector4Dot( Center, Plane );

    // Project the axes of the box onto the normal of the plane.  Half the
    // length of the projection (sometime called the "radius") is equal to
    // h(u) * abs(n dot b(u))) + h(v) * abs(n dot b(v)) + h(w) * abs(n dot b(w))
    // where h(i) are extents of the box, n is the plane normal, and b(i) are the 
    // axes of the box.
    DirectX::XMVECTOR Radius = DirectX::XMVector3Dot( Plane, Axis0 );
    Radius = DirectX::XMVectorInsert( Radius, DirectX::XMVector3Dot( Plane, Axis1 ), 0, 0, 1, 0, 0 );
    Radius = DirectX::XMVectorInsert( Radius, DirectX::XMVector3Dot( Plane, Axis2 ), 0, 0, 0, 1, 0 );
    Radius = DirectX::XMVector3Dot( Extents, DirectX::XMVectorAbs( Radius ) );

    // Outside the plane?
    Outside = DirectX::XMVectorGreater( Dist, Radius );

    // Fully inside the plane?
    Inside = DirectX::XMVectorLess( Dist, -Radius );
}



//-----------------------------------------------------------------------------
// Test an oriented box vs 6 planes (typically forming a frustum).
// Return values: 0 = no intersection, 
//                1 = may be intersecting, 
//                2 = box is inside all planes
//-----------------------------------------------------------------------------
INT IntersectOrientedBox6Planes( const OrientedBox* pVolume, DirectX::FXMVECTOR Plane0, DirectX::FXMVECTOR Plane1, DirectX::FXMVECTOR Plane2,
                                 CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 )
{
    XMASSERT( pVolume );

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );
    DirectX::XMVECTOR BoxOrientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( BoxOrientation ) );

    // Set w of the center to one so we can dot4 with a plane.
    Center = DirectX::XMVectorInsert( Center, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1 );

    // Build the 3x3 rotation matrix that defines the box axes.
    DirectX::XMMATRIX R = DirectX::XMMatrixRotationQuaternion( BoxOrientation );

    DirectX::XMVECTOR Outside, Inside;

    // Test against each plane.
    FastIntersectOrientedBoxPlane( Center, Extents, R.r[0], R.r[1], R.r[2], Plane0, Outside, Inside );

    DirectX::XMVECTOR AnyOutside = Outside;
    DirectX::XMVECTOR AllInside = Inside;

    FastIntersectOrientedBoxPlane( Center, Extents, R.r[0], R.r[1], R.r[2], Plane1, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectOrientedBoxPlane( Center, Extents, R.r[0], R.r[1], R.r[2], Plane2, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectOrientedBoxPlane( Center, Extents, R.r[0], R.r[1], R.r[2], Plane3, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectOrientedBoxPlane( Center, Extents, R.r[0], R.r[1], R.r[2], Plane4, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectOrientedBoxPlane( Center, Extents, R.r[0], R.r[1], R.r[2], Plane5, Outside, Inside );
    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    // If the box is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( AnyOutside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the box is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( AllInside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The box is not inside all planes or outside a plane, it may intersect.
    return 1;
}



//-----------------------------------------------------------------------------
static inline void FastIntersectFrustumPlane( FXMVECTOR Point0, DirectX::FXMVECTOR Point1, DirectX::FXMVECTOR Point2, CXMVECTOR Point3,
                                      CXMVECTOR Point4, CXMVECTOR Point5, CXMVECTOR Point6, CXMVECTOR Point7,
                                      CXMVECTOR Plane, DirectX::XMVECTOR& Outside, DirectX::XMVECTOR& Inside )
{
    // Find the min/max projection of the frustum onto the plane normal.
    DirectX::XMVECTOR Min, Max, Dist;

    Min = Max = DirectX::XMVector3Dot( Plane, Point0 );

    Dist = DirectX::XMVector3Dot( Plane, Point1 );
    Min = DirectX::XMVectorMin( Min, Dist );
    Max = DirectX::XMVectorMax( Max, Dist );

    Dist = DirectX::XMVector3Dot( Plane, Point2 );
    Min = DirectX::XMVectorMin( Min, Dist );
    Max = DirectX::XMVectorMax( Max, Dist );

    Dist = DirectX::XMVector3Dot( Plane, Point3 );
    Min = DirectX::XMVectorMin( Min, Dist );
    Max = DirectX::XMVectorMax( Max, Dist );

    Dist = DirectX::XMVector3Dot( Plane, Point4 );
    Min = DirectX::XMVectorMin( Min, Dist );
    Max = DirectX::XMVectorMax( Max, Dist );

    Dist = DirectX::XMVector3Dot( Plane, Point5 );
    Min = DirectX::XMVectorMin( Min, Dist );
    Max = DirectX::XMVectorMax( Max, Dist );

    Dist = DirectX::XMVector3Dot( Plane, Point6 );
    Min = DirectX::XMVectorMin( Min, Dist );
    Max = DirectX::XMVectorMax( Max, Dist );

    Dist = DirectX::XMVector3Dot( Plane, Point7 );
    Min = DirectX::XMVectorMin( Min, Dist );
    Max = DirectX::XMVectorMax( Max, Dist );

    DirectX::XMVECTOR PlaneDist = -DirectX::XMVectorSplatW( Plane );

    // Outside the plane?
    Outside = DirectX::XMVectorGreater( Min, PlaneDist );

    // Fully inside the plane?
    Inside = DirectX::XMVectorLess( Max, PlaneDist );
}



//-----------------------------------------------------------------------------
// Test a frustum vs 6 planes (typically forming another frustum).
// Return values: 0 = no intersection, 
//                1 = may be intersecting, 
//                2 = frustum is inside all planes
//-----------------------------------------------------------------------------
INT IntersectFrustum6Planes( const Frustum* pVolume, DirectX::FXMVECTOR Plane0, DirectX::FXMVECTOR Plane1, DirectX::FXMVECTOR Plane2,
                             CXMVECTOR Plane3, CXMVECTOR Plane4, CXMVECTOR Plane5 )
{
    XMASSERT( pVolume );

    // Load origin and orientation of the frustum.
    DirectX::XMVECTOR Origin = XMLoadFloat3( &pVolume->Origin );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Set w of the origin to one so we can dot4 with a plane.
    Origin = DirectX::XMVectorInsert( Origin, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1 );

    // Build the corners of the frustum (in world space).
    DirectX::XMVECTOR RightTop = DirectX::XMVectorSet( pVolume->RightSlope, pVolume->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR RightBottom = DirectX::XMVectorSet( pVolume->RightSlope, pVolume->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftTop = DirectX::XMVectorSet( pVolume->LeftSlope, pVolume->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftBottom = DirectX::XMVectorSet( pVolume->LeftSlope, pVolume->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR Near = DirectX::XMVectorSet( pVolume->Near, pVolume->Near, pVolume->Near, 0.0f );
    DirectX::XMVECTOR Far = DirectX::XMVectorSet( pVolume->Far, pVolume->Far, pVolume->Far, 0.0f );

    RightTop = DirectX::XMVector3Rotate( RightTop, Orientation );
    RightBottom = DirectX::XMVector3Rotate( RightBottom, Orientation );
    LeftTop = DirectX::XMVector3Rotate( LeftTop, Orientation );
    LeftBottom = DirectX::XMVector3Rotate( LeftBottom, Orientation );

    DirectX::XMVECTOR Corners0 = Origin + RightTop * Near;
    DirectX::XMVECTOR Corners1 = Origin + RightBottom * Near;
    DirectX::XMVECTOR Corners2 = Origin + LeftTop * Near;
    DirectX::XMVECTOR Corners3 = Origin + LeftBottom * Near;
    DirectX::XMVECTOR Corners4 = Origin + RightTop * Far;
    DirectX::XMVECTOR Corners5 = Origin + RightBottom * Far;
    DirectX::XMVECTOR Corners6 = Origin + LeftTop * Far;
    DirectX::XMVECTOR Corners7 = Origin + LeftBottom * Far;

    DirectX::XMVECTOR Outside, Inside;

    // Test against each plane.
    FastIntersectFrustumPlane( Corners0, Corners1, Corners2, Corners3, 
                               Corners4, Corners5, Corners6, Corners7, 
                               Plane0, Outside, Inside );

    DirectX::XMVECTOR AnyOutside = Outside;
    DirectX::XMVECTOR AllInside = Inside;

    FastIntersectFrustumPlane( Corners0, Corners1, Corners2, Corners3, 
                               Corners4, Corners5, Corners6, Corners7, 
                               Plane1, Outside, Inside );

    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectFrustumPlane( Corners0, Corners1, Corners2, Corners3, 
                               Corners4, Corners5, Corners6, Corners7, 
                               Plane2, Outside, Inside );

    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectFrustumPlane( Corners0, Corners1, Corners2, Corners3, 
                               Corners4, Corners5, Corners6, Corners7, 
                               Plane3, Outside, Inside );

    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectFrustumPlane( Corners0, Corners1, Corners2, Corners3, 
                               Corners4, Corners5, Corners6, Corners7, 
                               Plane4, Outside, Inside );

    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    FastIntersectFrustumPlane( Corners0, Corners1, Corners2, Corners3, 
                               Corners4, Corners5, Corners6, Corners7, 
                               Plane5, Outside, Inside );

    AnyOutside = DirectX::XMVectorOrInt( AnyOutside, Outside );
    AllInside = DirectX::XMVectorAndInt( AllInside, Inside );

    // If the frustum is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( AnyOutside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the frustum is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( AllInside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The frustum is not inside all planes or outside a plane, it may intersect.
    return 1;
}



//-----------------------------------------------------------------------------
INT IntersectTrianglePlane( FXMVECTOR V0, DirectX::FXMVECTOR V1, DirectX::FXMVECTOR V2, CXMVECTOR Plane )
{
    DirectX::XMVECTOR One = DirectX::XMVectorSplatOne();

    XMASSERT( XMPlaneIsUnit( Plane ) );

    // Set w of the points to one so we can dot4 with a plane.
    DirectX::XMVECTOR TV0 = DirectX::XMVectorInsert(V0, One, 0, 0, 0, 0, 1);
    DirectX::XMVECTOR TV1 = DirectX::XMVectorInsert(V1, One, 0, 0, 0, 0, 1);
    DirectX::XMVECTOR TV2 = DirectX::XMVectorInsert(V2, One, 0, 0, 0, 0, 1);

    DirectX::XMVECTOR Outside, Inside;
    FastIntersectTrianglePlane( TV0, TV1, TV2, Plane, Outside, Inside );

    // If the triangle is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the triangle is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( Inside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The triangle is not inside all planes or outside a plane it intersects.
    return 1;
}



//-----------------------------------------------------------------------------
INT IntersectSpherePlane( const Sphere* pVolume, DirectX::FXMVECTOR Plane )
{
    XMASSERT( pVolume );
    XMASSERT( XMPlaneIsUnit( Plane ) );

    // Load the sphere.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Radius = DirectX::XMVectorReplicatePtr( &pVolume->Radius );

    // Set w of the center to one so we can dot4 with a plane.
    Center = DirectX::XMVectorInsert( Center, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1 );

    DirectX::XMVECTOR Outside, Inside;
    FastIntersectSpherePlane( Center, Radius, Plane, Outside, Inside );

    // If the sphere is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the sphere is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( Inside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The sphere is not inside all planes or outside a plane it intersects.
    return 1;
}



//-----------------------------------------------------------------------------
INT IntersectAxisAlignedBoxPlane( const AxisAlignedBox* pVolume, DirectX::FXMVECTOR Plane )
{
    XMASSERT( pVolume );
    XMASSERT( XMPlaneIsUnit( Plane ) );

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );

    // Set w of the center to one so we can dot4 with a plane.
    Center = DirectX::XMVectorInsert( Center, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1);

    DirectX::XMVECTOR Outside, Inside;
    FastIntersectAxisAlignedBoxPlane( Center, Extents, Plane, Outside, Inside );

    // If the box is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the box is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( Inside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The box is not inside all planes or outside a plane it intersects.
    return 1;
}



//-----------------------------------------------------------------------------
INT IntersectOrientedBoxPlane( const OrientedBox* pVolume, DirectX::FXMVECTOR Plane )
{
    XMASSERT( pVolume );
    XMASSERT( XMPlaneIsUnit( Plane ) );

    // Load the box.
    DirectX::XMVECTOR Center = XMLoadFloat3( &pVolume->Center );
    DirectX::XMVECTOR Extents = XMLoadFloat3( &pVolume->Extents );
    DirectX::XMVECTOR BoxOrientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( BoxOrientation ) );

    // Set w of the center to one so we can dot4 with a plane.
    Center = DirectX::XMVectorInsert( Center, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1);

    // Build the 3x3 rotation matrix that defines the box axes.
    DirectX::XMMATRIX R = DirectX::XMMatrixRotationQuaternion( BoxOrientation );

    DirectX::XMVECTOR Outside, Inside;
    FastIntersectOrientedBoxPlane( Center, Extents, R.r[0], R.r[1], R.r[2], Plane, Outside, Inside );

    // If the box is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the box is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( Inside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The box is not inside all planes or outside a plane it intersects.
    return 1;
}



//-----------------------------------------------------------------------------
INT IntersectFrustumPlane( const Frustum* pVolume, DirectX::FXMVECTOR Plane )
{
    XMASSERT( pVolume );
    XMASSERT( XMPlaneIsUnit( Plane ) );

    // Load origin and orientation of the frustum.
    DirectX::XMVECTOR Origin = XMLoadFloat3( &pVolume->Origin );
    DirectX::XMVECTOR Orientation = XMLoadFloat4( &pVolume->Orientation );

    XMASSERT( XMQuaternionIsUnit( Orientation ) );

    // Set w of the origin to one so we can dot4 with a plane.
    Origin = DirectX::XMVectorInsert( Origin, DirectX::XMVectorSplatOne(), 0, 0, 0, 0, 1);

    // Build the corners of the frustum (in world space).
    DirectX::XMVECTOR RightTop = DirectX::XMVectorSet( pVolume->RightSlope, pVolume->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR RightBottom = DirectX::XMVectorSet( pVolume->RightSlope, pVolume->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftTop = DirectX::XMVectorSet( pVolume->LeftSlope, pVolume->TopSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR LeftBottom = DirectX::XMVectorSet( pVolume->LeftSlope, pVolume->BottomSlope, 1.0f, 0.0f );
    DirectX::XMVECTOR Near = DirectX::XMVectorSet( pVolume->Near, pVolume->Near, pVolume->Near, 0.0f );
    DirectX::XMVECTOR Far = DirectX::XMVectorSet( pVolume->Far, pVolume->Far, pVolume->Far, 0.0f );

    RightTop = DirectX::XMVector3Rotate( RightTop, Orientation );
    RightBottom = DirectX::XMVector3Rotate( RightBottom, Orientation );
    LeftTop = DirectX::XMVector3Rotate( LeftTop, Orientation );
    LeftBottom = DirectX::XMVector3Rotate( LeftBottom, Orientation );

    DirectX::XMVECTOR Corners0 = Origin + RightTop * Near;
    DirectX::XMVECTOR Corners1 = Origin + RightBottom * Near;
    DirectX::XMVECTOR Corners2 = Origin + LeftTop * Near;
    DirectX::XMVECTOR Corners3 = Origin + LeftBottom * Near;
    DirectX::XMVECTOR Corners4 = Origin + RightTop * Far;
    DirectX::XMVECTOR Corners5 = Origin + RightBottom * Far;
    DirectX::XMVECTOR Corners6 = Origin + LeftTop * Far;
    DirectX::XMVECTOR Corners7 = Origin + LeftBottom * Far;

    DirectX::XMVECTOR Outside, Inside;
    FastIntersectFrustumPlane( Corners0, Corners1, Corners2, Corners3, 
                               Corners4, Corners5, Corners6, Corners7, 
                               Plane, Outside, Inside );

    // If the frustum is outside any plane it is outside.
    if ( DirectX::XMVector4EqualInt( Outside, DirectX::XMVectorTrueInt() ) )
        return 0;

    // If the frustum is inside all planes it is inside.
    if ( DirectX::XMVector4EqualInt( Inside, DirectX::XMVectorTrueInt() ) )
        return 2;

    // The frustum is not inside all planes or outside a plane it intersects.
    return 1;
}

}; // namespace
