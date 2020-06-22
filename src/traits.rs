use na;
use na::*;
use na::storage::Storage;


pub trait JoinVec<T, U>{
    type Output;
    fn join(self, u: U) -> Self::Output;
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
        Vector3::new(self, v[0].inlined_clone(), v[1].inlined_clone())
    }
}

impl<T: ::BaseNum, S: Storage<T,U3,U1>> JoinVec<T,Vector<T, na::U3, S>> for T{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector<T, na::U3, S>) -> Vector4<T>{
        Vector4::new(self, v[0].inlined_clone(), v[1].inlined_clone(), v[2].inlined_clone())
    }
}

impl<'a, T, S> JoinVec<T,T> for Matrix<T,na::U1,na::U1,S>
    where T: ::BaseNum,
          S: Storage<T, na::U1, na::U1>
{
    type Output = Vector2<T>;
    #[inline]
    fn join(self, v: T) -> Vector2<T>{
        Vector2::new(self[0].inlined_clone(), v)
    }
}





impl<'a, T, S> JoinVec<T,T> for Matrix<T,na::U2,na::U1,S>
    where T: ::BaseNum,
          S: Storage<T,na::U2,na::U1>
{
    type Output = Vector3<T>;
    #[inline]
    fn join(self, v: T) -> Vector3<T>{
        Vector3::new(self[0].inlined_clone(), self[1].inlined_clone(), v)
    }
}

impl<'a, T, S> JoinVec<T,Vector<T, na::U2, S>> for Vector<T, na::U2, S>
    where T: ::BaseNum,
          S: Storage<T, na::U2, na::U1>,
{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: Vector<T, na::U2, S>) -> Vector4<T>{
        Vector4::new(self[0].inlined_clone(), self[1].inlined_clone(), v[0].inlined_clone(), v[1].inlined_clone())
    }
}





impl<'a, T, S> JoinVec<T,T> for Vector<T,na::U3,S>
    where T: ::BaseNum,
          S: Storage<T,na::U3,na::U1>
{
    type Output = Vector4<T>;
    #[inline]
    fn join(self, v: T) -> Vector4<T>{
        Vector4::new(self[0].inlined_clone(), self[1].inlined_clone(), self[2].inlined_clone(), v)
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
        Vector3::new(self[0].inlined_clone(), v[0].inlined_clone(), v[1].inlined_clone())
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
        Vector4::new(self[0].inlined_clone(), v[0].inlined_clone(), v[1].inlined_clone(), v[2].inlined_clone())
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
        Vector2::new(self.inlined_clone(), self)
    }
}

impl<T:Scalar> IntoVec<Vector2<T>> for [T;2]{
    #[inline]
    fn into_vec(self) -> Vector2<T>{
        let [x,y]: [T;2] = self;
        Vector2::new(x, y)
    }
}

impl<'a, T:Scalar> IntoVec<Vector2<T>> for &'a[T]{
    #[inline]
    fn into_vec(self) -> Vector2<T>{
        Vector2::new(self[0].inlined_clone(), self[1].inlined_clone())
    }
}

impl<T:Scalar> IntoVec<Vector3<T>> for T{
    #[inline]
    fn into_vec(self) -> Vector3<T>{
        Vector3::new(self.inlined_clone(), self.inlined_clone(), self)
    }
}

impl<T:Scalar> IntoVec<Vector3<T>> for [T;3]{
    #[inline]
    fn into_vec(self) -> Vector3<T>{
        let [x,y,z]: [T;3] = self;
        Vector3::new(x, y, z)
    }
}

impl<'a, T:Scalar> IntoVec<Vector3<T>> for &'a[T]{
    #[inline]
    fn into_vec(self) -> Vector3<T>{
        Vector3::new(self[0].inlined_clone(), self[1].inlined_clone(), self[2].inlined_clone())
    }
}

impl<T:Scalar> IntoVec<Vector4<T>> for T{
    #[inline]
    fn into_vec(self) -> Vector4<T>{
        Vector4::new(self.inlined_clone(), self.inlined_clone(), self.inlined_clone(), self)
    }
}

impl<T:Scalar> IntoVec<Vector4<T>> for [T;4]{
    #[inline]
    fn into_vec(self) -> Vector4<T>{
        let [x,y,z,w]: [T;4] = self.into();
        Vector4::new(x,y,z,w)
    }
}

impl<'a, T:Scalar> IntoVec<Vector4<T>> for &'a[T]{
    #[inline]
    fn into_vec(self) -> Vector4<T>{
        Vector4::new(self[0].inlined_clone(), self[1].inlined_clone(), self[2].inlined_clone(), self[3].inlined_clone())
    }
}


pub trait JoinPnt<T, U>{
    type Output;
    fn join(self, v: U) -> Self::Output;
}

impl<T: na::Scalar> JoinPnt<T,T> for T{
    type Output = Point2<T>;
    #[inline]
    fn join(self, v: T) -> Point2<T>{
        Point2::new(self, v)
    }
}

impl<T: na::Scalar> JoinPnt<T,Point2<T>> for T{
    type Output = Point3<T>;
    #[inline]
    fn join(self, v: Point2<T>) -> Point3<T>{
        let [y,z]: [T;2] = v.coords.into();
        Point3::new(self, y, z)
    }
}

impl<T: na::Scalar> JoinPnt<T,T> for Point2<T>{
    type Output = Point3<T>;
    #[inline]
    fn join(self, v: T) -> Point3<T>{
        let [x,y]: [T;2] = self.coords.into();
        Point3::new(x, y, v)
    }
}

impl<T: na::Scalar> JoinPnt<T,Point3<T>> for T{
    type Output = Point4<T>;
    #[inline]
    fn join(self, v: Point3<T>) -> Point4<T>{
        let [y,z,w]: [T;3] = v.coords.into();
        Point4::new(self, y, z, w)
    }
}

impl<T: na::Scalar> JoinPnt<T,T> for Point3<T>{
    type Output = Point4<T>;
    #[inline]
    fn join(self, v: T) -> Point4<T>{
        let [x,y,z]: [T;3] = self.coords.into();
        Point4::new(x, y, z, v)
    }
}

impl<T: na::Scalar> JoinPnt<T,Point2<T>> for Point2<T>{
    type Output = Point4<T>;
    #[inline]
    fn join(self, v: Point2<T>) -> Point4<T>{
        let [x,y]: [T;2] = self.coords.into();
        let [z,w]: [T;2] = v.coords.into();
        Point4::new(x,y,z,w)
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
        Point2::new(self.inlined_clone(), self)
    }
}

impl<T:Scalar> IntoPnt<Point2<T>> for [T;2]{
    #[inline]
    fn into_pnt(self) -> Point2<T>{
        let [x,y] = self;
        Point2::new(x, y)
    }
}

impl<'a, T:Scalar> IntoPnt<Point2<T>> for &'a[T]{
    #[inline]
    fn into_pnt(self) -> Point2<T>{
        Point2::new(self[0].inlined_clone(), self[1].inlined_clone())
    }
}

impl<T:Scalar> IntoPnt<Point3<T>> for T{
    #[inline]
    fn into_pnt(self) -> Point3<T>{
        Point3::new(self.inlined_clone(), self.inlined_clone(), self)
    }
}

impl<T:Scalar> IntoPnt<Point3<T>> for [T;3]{
    #[inline]
    fn into_pnt(self) -> Point3<T>{
        let [x,y,z] = self;
        Point3::new(x,y,z)
    }
}

impl<'a, T:Scalar> IntoPnt<Point3<T>> for &'a[T]{
    #[inline]
    fn into_pnt(self) -> Point3<T>{
        Point3::new(self[0].inlined_clone(), self[1].inlined_clone(), self[2].inlined_clone())
    }
}

impl<T:Scalar> IntoPnt<Point4<T>> for T{
    #[inline]
    fn into_pnt(self) -> Point4<T>{
        Point4::new(self.inlined_clone(), self.inlined_clone(), self.inlined_clone(), self)
    }
}

impl<T:Scalar> IntoPnt<Point4<T>> for [T;4]{
    #[inline]
    fn into_pnt(self) -> Point4<T>{
        let [x,y,z,w] = self;
        Point4::new(x,y,z,w)
    }
}

impl<'a, T:Scalar> IntoPnt<Point4<T>> for &'a[T]{
    #[inline]
    fn into_pnt(self) -> Point4<T>{
        Point4::new(self[0].inlined_clone(), self[1].inlined_clone(), self[2].inlined_clone(), self[3].inlined_clone())
    }
}