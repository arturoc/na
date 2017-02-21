use na::*;
use typenum::*;
use generic_array::*;
use std::ops::Mul;

pub trait JoinVec<T: ::BaseNum, U>{
    type Output: ::NumVec<T>;
    fn join(self, U) -> Self::Output;
}

impl<T: ::BaseNum> JoinVec<T,T> for T{
    type Output = Vector2<T>;
    #[inline]
    fn join(self, v: T) -> Vector2<T>{
        Vector2::new(self, v)
    }
}

impl<T: ::BaseNum> JoinVec<T,Vector2<T>> for T{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: Vector2<T>) -> Vector3<T>{
        Vector3::new(self, v.x, v.y)
    }
}

impl<T: ::BaseNum> JoinVec<T,T> for Vector2<T>{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: T) -> Vector3<T>{
        Vector3::new(self.x, self.y, v)
    }
}

impl<T: ::BaseNum> JoinVec<T,Vector3<T>> for T{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector3<T>) -> Vector4<T>{
        Vector4::new(self, v.x, v.y, v.z)
    }
}

impl<T: ::BaseNum> JoinVec<T,T> for Vector3<T>{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: T) -> Vector4<T>{
        Vector4::new(self.x, self.y, self.z, v)
    }
}

impl<T: ::BaseNum> JoinVec<T,Vector2<T>> for Vector2<T>{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector2<T>) -> Vector4<T>{
        Vector4::new(self.x, self.y, v.x, v.y)
    }
}

pub trait IntoVec<V>{
    fn into_vec(self) -> V;
}

impl<V> IntoVec<V> for V{
    #[inline]
    fn into_vec(self) -> V{
        self
    }
}

impl<T:Scalar,D: DimName> IntoVec<VectorN<T,D>> for T
    where <D as DimName>::Value: Mul<UInt<UTerm, bit::B1>>,
          <<D as DimName>::Value as Mul<UInt<UTerm, B1>>>::Output: ArrayLength<T>{
    #[inline]
    fn into_vec(self) -> VectorN<T,D>{
        VectorN::from_element(self)
    }
}

pub trait JoinPnt<T: ::BaseNum, U>{
    type Output: ::NumPnt<T>;
    fn join(self, U) -> Self::Output;
}

impl<T: ::BaseNum> JoinPnt<T,T> for T{
    type Output = Point2<T>;
    #[inline]
    fn join(self, v: T) -> Point2<T>{
        Point2::new(self, v)
    }
}

impl<T: ::BaseNum> JoinPnt<T,Point2<T>> for T{
    type Output = Point3<T>;
    #[inline]
    fn join(self, v: Point2<T>) -> Point3<T>{
        Point3::new(self, v.x, v.y)
    }
}

impl<T: ::BaseNum> JoinPnt<T,T> for Point2<T>{
    type Output = Point3<T>;
    #[inline]
    fn join(self, v: T) -> Point3<T>{
        Point3::new(self.x, self.y, v)
    }
}

impl<T: ::BaseNum> JoinPnt<T,Point3<T>> for T{
    type Output = Point4<T>;
    #[inline]
    fn join(self, v: Point3<T>) -> Point4<T>{
        Point4::new(self, v.x, v.y, v.z)
    }
}

impl<T: ::BaseNum> JoinPnt<T,T> for Point3<T>{
    type Output = Point4<T>;
    #[inline]
    fn join(self, v: T) -> Point4<T>{
        Point4::new(self.x, self.y, self.z, v)
    }
}

impl<T: ::BaseNum> JoinPnt<T,Point2<T>> for Point2<T>{
    type Output = Point4<T>;
    #[inline]
    fn join(self, v: Point2<T>) -> Point4<T>{
        Point4::new(self.x, self.y, v.x, v.y)
    }
}

pub trait IntoPnt<V>{
    fn into_pnt(self) -> V;
}

impl<V> IntoPnt<V> for V{
    #[inline]
    fn into_pnt(self) -> V{
        self
    }
}

impl<T:Scalar> IntoPnt<Point2<T>> for T{
    #[inline]
    fn into_pnt(self) -> Point2<T>{
        Point2::new(self, self)
    }
}

impl<T:Scalar> IntoPnt<Point3<T>> for T{
    #[inline]
    fn into_pnt(self) -> Point3<T>{
        Point3::new(self, self, self)
    }
}

impl<T:Scalar> IntoPnt<Point4<T>> for T{
    #[inline]
    fn into_pnt(self) -> Point4<T>{
        Point4::new(self, self, self, self)
    }
}
