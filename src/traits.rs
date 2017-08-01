use na;
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


impl<'a, T, RStride, CStride, Alloc> JoinVec<T,T> for Matrix<T,na::U1,na::U1,SliceStorage<'a,T,na::U1,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U1,na::U1>{
    type Output = Vector2<T>;
    #[inline]
    fn join(self, v: T) -> Vector2<T>{
        Vector2::new(self[0], v)
    }
}

impl<'a, T, RStride, CStride, Alloc> JoinVec<T,Vector2<T>> for Matrix<T,na::U1,na::U1,SliceStorage<'a,T,na::U1,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U1,na::U1>{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: Vector2<T>) -> Vector3<T>{
        Vector3::new(self[0], v.x, v.y)
    }
}

impl<'a, T, RStride, CStride, Alloc> JoinVec<T,Vector3<T>> for Matrix<T,na::U1,na::U1,SliceStorage<'a,T,na::U1,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U1,na::U1>{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector3<T>) -> Vector4<T>{
        Vector4::new(self[0], v.x, v.y, v.z)
    }
}





impl<'a, T, RStride, CStride, Alloc> JoinVec<T,T> for Matrix<T,na::U2,na::U1,SliceStorage<'a,T,na::U2,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U2,na::U1>{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: T) -> Vector3<T>{
        Vector3::new(self[0], self[1], v)
    }
}

impl<'a, T, RStride, CStride, Alloc> JoinVec<T,Vector2<T>> for Matrix<T,na::U2,na::U1,SliceStorage<'a,T,na::U2,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U2,na::U1>{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector2<T>) -> Vector4<T>{
        Vector4::new(self[0], self[1], v.x, v.y)
    }
}





impl<'a, T, RStride, CStride, Alloc> JoinVec<T,T> for Matrix<T,na::U3,na::U1,SliceStorage<'a,T,na::U3,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U3,na::U1>{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: T) -> Vector4<T>{
        Vector4::new(self[0], self[1], self[2], v)
    }
}


impl<'a, T, RStride, CStride, Alloc> JoinVec<T,T> for Matrix<T,na::U1,na::U1,SliceStorageMut<'a,T,na::U1,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U1,na::U1>{
    type Output = Vector2<T>;
    #[inline]
    fn join(self, v: T) -> Vector2<T>{
        Vector2::new(self[0], v)
    }
}

impl<'a, T, RStride, CStride, Alloc> JoinVec<T,Vector2<T>> for Matrix<T,na::U1,na::U1,SliceStorageMut<'a,T,na::U1,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U1,na::U1>{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: Vector2<T>) -> Vector3<T>{
        Vector3::new(self[0], v.x, v.y)
    }
}

impl<'a, T, RStride, CStride, Alloc> JoinVec<T,Vector3<T>> for Matrix<T,na::U1,na::U1,SliceStorageMut<'a,T,na::U1,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U1,na::U1>{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector3<T>) -> Vector4<T>{
        Vector4::new(self[0], v.x, v.y, v.z)
    }
}





impl<'a, T, RStride, CStride, Alloc> JoinVec<T,T> for Matrix<T,na::U2,na::U1,SliceStorageMut<'a,T,na::U2,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U2,na::U1>{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: T) -> Vector3<T>{
        Vector3::new(self[0], self[1], v)
    }
}

impl<'a, T, RStride, CStride, Alloc> JoinVec<T,Vector2<T>> for Matrix<T,na::U2,na::U1,SliceStorageMut<'a,T,na::U2,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U2,na::U1>{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector2<T>) -> Vector4<T>{
        Vector4::new(self[0], self[1], v.x, v.y)
    }
}





impl<'a, T, RStride, CStride, Alloc> JoinVec<T,T> for Matrix<T,na::U3,na::U1,SliceStorageMut<'a,T,na::U3,na::U1,RStride,CStride,Alloc>>
    where T: ::BaseNum,
          RStride: na::Dim,
          CStride: na::Dim,
          Alloc: allocator::Allocator<T,na::U3,na::U1>{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: T) -> Vector4<T>{
        Vector4::new(self[0], self[1], self[2], v)
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

impl<T,D> IntoVec<VectorN<T,D>> for T
    where T:Scalar,
          D: DimName,
          <D as DimName>::Value: Mul<UInt<UTerm, bit::B1>>,
          <<D as DimName>::Value as Mul<UInt<UTerm, B1>>>::Output: ArrayLength<T>{
    #[inline]
    fn into_vec(self) -> VectorN<T,D>{
        VectorN::from_element(self)
    }
}

impl<T:Scalar> IntoVec<Vector2<T>> for [T;2]{
    #[inline]
    fn into_vec(self) -> Vector2<T>{
        Vector2::new(self[0], self[1])
    }
}

impl<'a, T:Scalar> IntoVec<Vector2<T>> for &'a[T]{
    #[inline]
    fn into_vec(self) -> Vector2<T>{
        Vector2::new(self[0], self[1])
    }
}

impl<T:Scalar> IntoVec<Vector3<T>> for [T;3]{
    #[inline]
    fn into_vec(self) -> Vector3<T>{
        Vector3::new(self[0], self[1], self[2])
    }
}

impl<'a, T:Scalar> IntoVec<Vector3<T>> for &'a[T]{
    #[inline]
    fn into_vec(self) -> Vector3<T>{
        Vector3::new(self[0], self[1], self[2])
    }
}

impl<T:Scalar> IntoVec<Vector4<T>> for [T;4]{
    #[inline]
    fn into_vec(self) -> Vector4<T>{
        Vector4::new(self[0], self[1], self[2], self[3])
    }
}

impl<'a, T:Scalar> IntoVec<Vector4<T>> for &'a[T]{
    #[inline]
    fn into_vec(self) -> Vector4<T>{
        Vector4::new(self[0], self[1], self[2], self[3])
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

impl<T:Scalar> IntoPnt<Point2<T>> for [T;2]{
    #[inline]
    fn into_pnt(self) -> Point2<T>{
        Point2::new(self[0], self[1])
    }
}

impl<'a, T:Scalar> IntoPnt<Point2<T>> for &'a[T]{
    #[inline]
    fn into_pnt(self) -> Point2<T>{
        Point2::new(self[0], self[1])
    }
}

impl<T:Scalar> IntoPnt<Point3<T>> for [T;3]{
    #[inline]
    fn into_pnt(self) -> Point3<T>{
        Point3::new(self[0], self[1], self[2])
    }
}

impl<'a, T:Scalar> IntoPnt<Point3<T>> for &'a[T]{
    #[inline]
    fn into_pnt(self) -> Point3<T>{
        Point3::new(self[0], self[1], self[2])
    }
}

impl<T:Scalar> IntoPnt<Point4<T>> for [T;4]{
    #[inline]
    fn into_pnt(self) -> Point4<T>{
        Point4::new(self[0], self[1], self[2], self[3])
    }
}

impl<'a, T:Scalar> IntoPnt<Point4<T>> for &'a[T]{
    #[inline]
    fn into_pnt(self) -> Point4<T>{
        Point4::new(self[0], self[1], self[2], self[3])
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
