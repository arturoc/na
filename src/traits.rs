use na;
use na::*;
use na::storage::Storage;


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

impl<T: ::BaseNum, S: Storage<T,U2,U1>> JoinVec<T,Vector<T, na::U2, S>> for T{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: Vector<T, na::U2, S>) -> Vector3<T>{
        Vector3::new(self, v[0], v[1])
    }
}

impl<T: ::BaseNum, S: Storage<T,U3,U1>> JoinVec<T,Vector<T, na::U3, S>> for T{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector<T, na::U3, S>) -> Vector4<T>{
        Vector4::new(self, v[0], v[1], v[2])
    }
}

impl<'a, T, S> JoinVec<T,T> for Matrix<T,na::U1,na::U1,S>
    where T: ::BaseNum,
          S: Storage<T, na::U1, na::U1>
{
    type Output = Vector2<T>;
    #[inline]
    fn join(self, v: T) -> Vector2<T>{
        Vector2::new(self[0], v)
    }
}





impl<'a, T, S> JoinVec<T,T> for Matrix<T,na::U2,na::U1,S>
    where T: ::BaseNum,
          S: Storage<T,na::U2,na::U1>
{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: T) -> Vector3<T>{
        Vector3::new(self[0], self[1], v)
    }
}

impl<'a, T, S> JoinVec<T,Vector<T, na::U2, S>> for Vector<T, na::U2, S>
    where T: ::BaseNum,
          S: Storage<T, na::U2, na::U1>,
{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector<T, na::U2, S>) -> Vector4<T>{
        Vector4::new(self[0], self[1], v[0], v[1])
    }
}





impl<'a, T, S> JoinVec<T,T> for Vector<T,na::U3,S>
    where T: ::BaseNum,
          S: Storage<T,na::U3,na::U1>
{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: T) -> Vector4<T>{
        Vector4::new(self[0], self[1], self[2], v)
    }
}

impl<'a, T, S1, S2> JoinVec<T,Vector<T,na::U2,S2>> for Vector<T,na::U1,S1>
    where T: ::BaseNum,
          S1: Storage<T,na::U1,na::U1>,
          S2: Storage<T,na::U2,na::U1>,
{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: Vector<T,na::U2,S2>) -> Vector3<T>{
        Vector3::new(self[0], v[0], v[1])
    }
}

impl<'a, T, S1, S3> JoinVec<T,Vector<T,na::U3,S3>> for Matrix<T,na::U1,na::U1,S1>
    where T: ::BaseNum,
          S1: Storage<T,na::U1,na::U1>,
          S3: Storage<T,na::U3,na::U1>,
{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector<T,na::U3,S3>) -> Vector4<T>{
        Vector4::new(self[0], v[0], v[1], v[2])
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

impl<T:Scalar> IntoVec<Vector2<T>> for T{
    #[inline]
    fn into_vec(self) -> Vector2<T>{
        Vector2::new(self, self)
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

impl<T:Scalar> IntoVec<Vector3<T>> for T{
    #[inline]
    fn into_vec(self) -> Vector3<T>{
        Vector3::new(self, self, self)
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

impl<T:Scalar> IntoVec<Vector4<T>> for T{
    #[inline]
    fn into_vec(self) -> Vector4<T>{
        Vector4::new(self, self, self, self)
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

impl<T:Scalar> IntoPnt<Point2<T>> for T{
    #[inline]
    fn into_pnt(self) -> Point2<T>{
        Point2::new(self, self)
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

impl<T:Scalar> IntoPnt<Point3<T>> for T{
    #[inline]
    fn into_pnt(self) -> Point3<T>{
        Point3::new(self, self, self)
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

impl<T:Scalar> IntoPnt<Point4<T>> for T{
    #[inline]
    fn into_pnt(self) -> Point4<T>{
        Point4::new(self, self, self, self)
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
