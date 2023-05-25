#![cfg_attr(feature = "unstable", feature(test))]

use nalgebra as na;
use num_traits as num;

pub type Vec1<T=f32> = na::Vector1<T>;
pub type Vec2<T=f32> = na::Vector2<T>;
pub type Vec3<T=f32> = na::Vector3<T>;
pub type Vec4<T=f32> = na::Vector4<T>;
pub type Vec5<T=f32> = na::Vector5<T>;
pub type Vec6<T=f32> = na::Vector6<T>;

pub type Pnt1<T=f32> = na::Point1<T>;
pub type Pnt2<T=f32> = na::Point2<T>;
pub type Pnt3<T=f32> = na::Point3<T>;
pub type Pnt4<T=f32> = na::Point4<T>;
pub type Pnt5<T=f32> = na::Point5<T>;
pub type Pnt6<T=f32> = na::Point6<T>;

pub type Mat1<T=f32> = na::Matrix1<T>;
pub type Mat2<T=f32> = na::Matrix2<T>;
pub type Mat3<T=f32> = na::Matrix3<T>;
pub type Mat4<T=f32> = na::Matrix4<T>;
pub type Mat5<T=f32> = na::Matrix5<T>;
pub type Mat6<T=f32> = na::Matrix6<T>;


pub type Quat<T=f32> = na::Quaternion<T>;
pub type UnitQuat<T=f32> = na::UnitQuaternion<T>;
pub type DualQuat<T=f32> = na::DualQuaternion<T>;
pub type UnitDualQuat<T=f32> = na::UnitDualQuaternion<T>;

pub type Isometry3<T=f32> = na::Isometry3<T>;
pub type Isometry2<T=f32> = na::Isometry2<T>;
pub type Rotation3<T=f32> = na::Rotation3<T>;
pub type Rotation2<T=f32> = na::Rotation2<T>;
pub type Translation3<T=f32> = na::Translation3<T>;
pub type Translation2<T=f32> = na::Translation2<T>;

pub use na::*;
pub use na::storage::{Storage, StorageMut};
use std::mem;
use std::ops::*;
pub use simba::*;

pub mod traits;

// vec constructors
#[inline]
pub fn vec2<T: Scalar>(x: T, y: T) -> Vector2<T>{
    Vector2::new(x,y)
}

#[inline]
pub fn vec3<T: Scalar>(x: T, y: T, z: T) -> Vector3<T>{
    Vector3::new(x,y,z)
}

#[inline]
pub fn vec4<T: Scalar>(x: T, y: T, z: T, w: T) -> Vector4<T>{
    Vector4::new(x,y,z,w)
}

#[inline]
pub fn vec5<T: Scalar>(x: T, y: T, z: T, w: T, a: T) -> Vector5<T>{
    Vector5::new(x,y,z,w,a)
}

#[inline]
pub fn vec6<T: Scalar>(x: T, y: T, z: T, w: T, a: T, b: T) -> Vector6<T>{
    Vector6::new(x,y,z,w,a,b)
}

#[inline]
pub fn pnt2<T: Scalar>(x: T, y: T) -> Point2<T>{
    Point2::new(x,y)
}

#[inline]
pub fn pnt3<T: Scalar>(x: T, y: T, z: T) -> Point3<T>{
    Point3::new(x,y,z)
}

#[inline]
pub fn pnt4<T: Scalar>(x: T, y: T, z: T, w: T) -> Point4<T>{
    Point4::new(x,y,z,w)
}

#[inline]
pub fn pnt5<T: Scalar>(x: T, y: T, z: T, w: T, a: T) -> Point5<T>{
    Point5::new(x,y,z,w,a)
}

#[inline]
pub fn pnt6<T: Scalar>(x: T, y: T, z: T, w: T, a: T, b: T) -> Point6<T>{
    Point6::new(x,y,z,w,a,b)
}

pub trait ToPnt<T>{
    fn to_pnt(self) -> T;
}

pub trait AsPnt<T>{
    fn as_pnt(&self) -> &T;
}

pub trait ToVec<T>{
    fn to_vec(self) -> T;
}

pub trait AsVec<T>{
    fn as_vec(&self) -> &T;
}

macro_rules! vec_to_pnt_impl{
    ($v: ident, $p: ident) => (
        impl<T: na::Scalar> ToPnt<$p<T>> for $v<T>{
            #[inline]
            fn to_pnt(self) -> $p<T>{
                $p::from(self)
            }
        }

        impl<T: na::Scalar> AsPnt<$p<T>> for $v<T>{
            #[inline]
            fn as_pnt(&self) -> &$p<T>{
                unsafe{ mem::transmute(self) }
            }
        }

        impl<T: na::Scalar> ToVec<$v<T>> for $p<T>{
            #[inline]
            fn to_vec(self) -> $v<T>{
                self.coords
            }
        }

        impl<T: na::Scalar> AsVec<$v<T>> for $p<T>{
            #[inline]
            fn as_vec(&self) -> &$v<T>{
                &self.coords
            }
        }
    )
}

vec_to_pnt_impl!(Vector1, Point1);
vec_to_pnt_impl!(Vector2, Point2);
vec_to_pnt_impl!(Vector3, Point3);
vec_to_pnt_impl!(Vector4, Point4);
vec_to_pnt_impl!(Vector5, Point5);
vec_to_pnt_impl!(Vector6, Point6);

pub trait ToMat<T>{
    fn to_mat(self) -> T;
}

pub trait AsMat<T>{
    fn as_mat(&self) -> &T;
}


impl<T: na::RealField> ToMat<Matrix4<T>> for Orthographic3<T>{
    fn to_mat(self) -> Matrix4<T>{
        self.to_homogeneous()
    }
}

impl<T: na::RealField> AsMat<Matrix4<T>> for Orthographic3<T>{
    fn as_mat(&self) -> &Matrix4<T>{
        self.as_matrix()
    }
}

pub trait FastInverse{
    fn fast_orthonormal_inverse(&self) -> Self;
    fn fast_affine_inverse(&self) -> Option<Self> where Self: Sized;
}

impl<T:na::RealField> FastInverse for Matrix4<T>{
    fn fast_orthonormal_inverse(&self) -> Matrix4<T>{
        let _3x3 = Matrix3::new(
            self.m11.clone(), self.m21.clone(), self.m31.clone(),
            self.m12.clone(), self.m22.clone(), self.m32.clone(),
            self.m13.clone(), self.m23.clone(), self.m33.clone(),
        );
        let pos = vec3(self.m14.clone(), self.m24.clone(), self.m34.clone());
        let pos = -_3x3 * pos;
        Matrix4::new(
            self.m11.clone(), self.m21.clone(), self.m31.clone(), pos.x.clone(),
            self.m12.clone(), self.m22.clone(), self.m32.clone(), pos.y.clone(),
            self.m13.clone(), self.m23.clone(), self.m33.clone(), pos.z.clone(),
            zero(),   zero(),   zero(),   one()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ,
        )
    }

    fn fast_affine_inverse(&self) -> Option<Matrix4<T>>{
        Matrix3::new(
            self.m11.clone(), self.m12.clone(), self.m13.clone(),
            self.m21.clone(), self.m22.clone(), self.m23.clone(),
            self.m31.clone(), self.m32.clone(), self.m33.clone(),
        ).try_inverse().map(|_3x3| {
            let pos = vec3(self.m14.clone(), self.m24.clone(), self.m34.clone());
            let pos = -&_3x3 * pos;
            Matrix4::new(
                _3x3.m11.clone(), _3x3.m12.clone(), _3x3.m13.clone(), pos.x.clone(),
                _3x3.m21.clone(), _3x3.m22.clone(), _3x3.m23.clone(), pos.y.clone(),
                _3x3.m31.clone(), _3x3.m32.clone(), _3x3.m33.clone(), pos.z.clone(),
                zero(),   zero(),   zero(),   one()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ,
            )
        })
    }
}

pub trait BaseNum: Scalar
    + num::Zero
    + num::One
    + Add<Self, Output = Self> + Sub<Self, Output = Self>
    + Mul<Self, Output = Self> + Div<Self, Output = Self>
    + Rem<Self, Output = Self>
    + AddAssign<Self>
    + SubAssign<Self>
    + MulAssign<Self>
    + DivAssign<Self>
    + RemAssign<Self>
    + PartialOrd
    + 'static
{}

impl BaseNum for i8 { }
impl BaseNum for i16 { }
impl BaseNum for i32 { }
impl BaseNum for i64 { }
impl BaseNum for isize { }
impl BaseNum for u8 { }
impl BaseNum for u16 { }
impl BaseNum for u32 { }
impl BaseNum for u64 { }
impl BaseNum for usize { }
impl BaseNum for f32 { }
impl BaseNum for f64 { }

pub trait BaseInt: BaseNum
    + Shl<Self, Output = Self>
    + ShlAssign<Self>
    + Shr<Self, Output=Self>
    + ShrAssign<Self>
{}

impl BaseInt for i8 { }
impl BaseInt for i16 { }
impl BaseInt for i32 { }
impl BaseInt for i64 { }
impl BaseInt for isize { }
impl BaseInt for u8 { }
impl BaseInt for u16 { }
impl BaseInt for u32 { }
impl BaseInt for u64 { }
impl BaseInt for usize { }

/*
 * Vector related traits.
 */
/// Trait grouping most common operations on vectors.
pub trait NumVec: Add<Self, Output = Self>
    + Sub<Self, Output = Self>
    + Mul<<Self as NumVec>::Field, Output = Self>
    + Div<<Self as NumVec>::Field, Output = Self>
    + MulAssign<<Self as NumVec>::Field>
    + DivAssign<<Self as NumVec>::Field>
    + AddAssign<Self>
    + SubAssign<Self>
    + PartialEq
    + Sized

{
    type Field: BaseNum;
}

/// Trait of vector with components implementing the `BaseFloat` trait.
pub trait FloatVec: NumVec + Neg<Output = Self>
where
    <Self as NumVec>::Field: na::RealField,
{
}

impl<N: BaseNum> NumVec for Vector1<N>{ type Field = N; }
impl<N: BaseNum> NumVec for Vector2<N>{ type Field = N; }
impl<N: BaseNum> NumVec for Vector3<N>{ type Field = N; }
impl<N: BaseNum> NumVec for Vector4<N>{ type Field = N; }
impl<N: BaseNum> NumVec for Vector5<N>{ type Field = N; }
impl<N: BaseNum> NumVec for Vector6<N>{ type Field = N; }

impl<N: BaseNum + na::RealField> FloatVec for Vector1<N>{}
impl<N: BaseNum + na::RealField> FloatVec for Vector2<N>{}
impl<N: BaseNum + na::RealField> FloatVec for Vector3<N>{}
impl<N: BaseNum + na::RealField> FloatVec for Vector4<N>{}
impl<N: BaseNum + na::RealField> FloatVec for Vector5<N>{}
impl<N: BaseNum + na::RealField> FloatVec for Vector6<N>{}

/*
 * Point related traits.
 */
/// Trait grouping most common operations on points.
pub trait NumPnt: Sub<Self, Output = <Self as NumPnt>::Coordinates>
    + Mul<<Self as NumPnt>::Field, Output = Self>
    + Div<<Self as NumPnt>::Field, Output = Self>
    + MulAssign<<Self as NumPnt>::Field>
    + DivAssign<<Self as NumPnt>::Field>
    + Add<<Self as NumPnt>::Coordinates, Output = Self>
    + Sub<<Self as NumPnt>::Coordinates, Output = Self>
    + AddAssign<<Self as NumPnt>::Coordinates>
    + SubAssign<<Self as NumPnt>::Coordinates>
    + PartialEq
    + Sized
{
    type Field: BaseNum;
    type Coordinates: std::ops::Index<usize, Output = Self::Field>
        + std::ops::IndexMut<usize>
        + Add<Self::Coordinates, Output = Self::Coordinates>
        + Sub<Self::Coordinates, Output = Self::Coordinates>
        + AddAssign<Self::Coordinates>
        + SubAssign<Self::Coordinates>
        + Mul<Self::Field, Output = Self::Coordinates>
        + Div<Self::Field, Output = Self::Coordinates>
        + MulAssign<Self::Field>
        + DivAssign<Self::Field>;

    fn coordinates(&self) -> Self::Coordinates;
    fn from_coordiantes(coords: Self::Coordinates) -> Self;
}

/// Trait of vector with components implementing the `RealField` trait.
pub trait FloatPnt: NumPnt + Neg<Output = Self>
where
    <Self as NumPnt>::Field: RealField + Neg<Output = <Self as NumPnt>::Field>,
    <Self as NumPnt>::Coordinates: Neg<Output = <Self as NumPnt>::Coordinates>
{
    fn origin() -> Self;
    fn distance(&self, b: &Self) -> Self::Field;
    fn distance_squared(&self, b: &Self) -> Self::Field;
}


impl<N: BaseNum> NumPnt for Point1<N>{
    type Field = N;
    type Coordinates = Vector1<N>;

    fn coordinates(&self) -> Self::Coordinates{
        self.clone().to_vec()
    }

    fn from_coordiantes(coords: Self::Coordinates) -> Self {
        Self::from(coords)
    }
}
impl<N: BaseNum> NumPnt for Point2<N>{
    type Field = N;
    type Coordinates = Vector2<N>;

    fn coordinates(&self) -> Self::Coordinates{
        self.clone().to_vec()
    }

    fn from_coordiantes(coords: Self::Coordinates) -> Self {
        Self::from(coords)
    }
}
impl<N: BaseNum> NumPnt for Point3<N>{
    type Field = N;
    type Coordinates = Vector3<N>;

    fn coordinates(&self) -> Self::Coordinates{
        self.clone().to_vec()
    }

    fn from_coordiantes(coords: Self::Coordinates) -> Self {
        Self::from(coords)
    }
}
impl<N: BaseNum> NumPnt for Point4<N>{
    type Field = N;
    type Coordinates = Vector4<N>;

    fn coordinates(&self) -> Self::Coordinates{
        self.clone().to_vec()
    }

    fn from_coordiantes(coords: Self::Coordinates) -> Self {
        Self::from(coords)
    }
}
impl<N: BaseNum> NumPnt for Point5<N>{
    type Field = N;
    type Coordinates = Vector5<N>;

    fn coordinates(&self) -> Self::Coordinates{
        self.clone().to_vec()
    }

    fn from_coordiantes(coords: Self::Coordinates) -> Self {
        Self::from(coords)
    }
}
impl<N: BaseNum> NumPnt for Point6<N>{
    type Field = N;
    type Coordinates = Vector6<N>;

    fn coordinates(&self) -> Self::Coordinates{
        self.clone().to_vec()
    }

    fn from_coordiantes(coords: Self::Coordinates) -> Self {
        Self::from(coords)
    }
}

impl<N: BaseNum + na::RealField> FloatPnt for Point1<N>{
    fn origin() -> Self{
        Point::origin()
    }

    fn distance(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance(self, b)
    }

    fn distance_squared(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance_squared(self, b)
    }
}
impl<N: BaseNum + na::RealField> FloatPnt for Point2<N>{
    fn origin() -> Self{
        Point::origin()
    }

    fn distance(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance(self, b)
    }

    fn distance_squared(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance_squared(self, b)
    }
}
impl<N: BaseNum + na::RealField> FloatPnt for Point3<N>{
    fn origin() -> Self{
        Point::origin()
    }

    fn distance(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance(self, b)
    }

    fn distance_squared(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance_squared(self, b)
    }
}
impl<N: BaseNum + na::RealField> FloatPnt for Point4<N>{
    fn origin() -> Self{
        Point::origin()
    }

    fn distance(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance(self, b)
    }

    fn distance_squared(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance_squared(self, b)
    }
}
impl<N: BaseNum + na::RealField> FloatPnt for Point5<N>{
    fn origin() -> Self{
        Point::origin()
    }

    fn distance(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance(self, b)
    }

    fn distance_squared(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance_squared(self, b)
    }
}
impl<N: BaseNum + na::RealField> FloatPnt for Point6<N>{
    fn origin() -> Self{
        Point::origin()
    }

    fn distance(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance(self, b)
    }

    fn distance_squared(&self, b: &Self) -> <Self as NumPnt>::Field {
        na::distance_squared(self, b)
    }
}

#[macro_export]
macro_rules! vec2{
    ($x: expr, $y: expr) => ({
        $crate::Vector2::new($x, $y)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoVec;
        let v: $crate::Vector2<_> = $v1.into_vec();
        v
    });
}

#[macro_export]
macro_rules! vec3{
    ($x: expr, $y: expr, $z: expr) => ({
        $crate::Vector3::new($x, $y, $z)
    });
    ($v1: expr, $v2: expr) => ({
        use $crate::traits::JoinVec;
        $v1.join($v2)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoVec;
        let v: $crate::Vector3<_> = $v1.into_vec();
        v
    });
}

#[macro_export]
macro_rules! vec4{
    ($x: expr, $y: expr, $z: expr, $w: expr) => ({
        $crate::Vector4::new($x, $y, $z, $w)
    });
    ($v1: expr, $v2: expr, $v3: expr) => ({
        use $crate::traits::JoinVec;
        $v1.join($v2).join($v3)
    });
    ($v1: expr, $v2: expr) => ({
        use $crate::traits::JoinVec;
        $v1.join($v2)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoVec;
        let v: $crate::Vector4<_> = $v1.into_vec();
        v
    });
}


#[macro_export]
macro_rules! pnt2{
    ($x: expr, $y: expr) => ({
        $crate::Point2::new($x, $y)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoPnt;
        let v: $crate::Point2<_> = $v1.into_pnt();
        v
    });
}

#[macro_export]
macro_rules! pnt3{
    ($x: expr, $y: expr, $z: expr) => ({
        $crate::Point3::new($x, $y, $z)
    });
    ($v1: expr, $v2: expr) => ({
        use $crate::traits::JoinPnt;
        $v1.join($v2)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoPnt;
        let v: $crate::Point3<_> = $v1.into_pnt();
        v
    });
}

#[macro_export]
macro_rules! pnt4{
    ($x: expr, $y: expr, $z: expr, $w: expr) => ({
        $crate::Point4::new($x, $y, $z, $w)
    });
    ($v1: expr, $v2: expr, $v3: expr) => ({
        use $crate::traits::JoinPnt;
        $v1.join($v2).join($v3)
    });
    ($v1: expr, $v2: expr) => ({
        use $crate::traits::JoinPnt;
        $v1.join($v2)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoPnt;
        let v: $crate::Vector4<_> = $v1.into_pnt();
        v
    });
}

pub trait Swizzles2<T: Scalar>{
    type Swizzle2;
    fn xy(&self) -> Self::Swizzle2;
    fn yx(&self) -> Self::Swizzle2;
}

pub trait Swizzles2Mut<T: Scalar>: Swizzles2<T>{
    fn set_xy(&mut self, right: &Self::Swizzle2);
    fn set_yx(&mut self, right: &Self::Swizzle2);
}

macro_rules! swizzles2_impl{
    ($v: ident, $o: ident) => (
        impl<T: Scalar> Swizzles2<T> for $v<T>{
            type Swizzle2 = $o<T>;
            fn xy(&self) -> $o<T>{
                $o::new(self.x.clone(), self.y.clone())
            }
            fn yx(&self) -> $o<T>{
                $o::new(self.y.clone(), self.x.clone())
            }
        }

        impl<T: Scalar> Swizzles2Mut<T> for $v<T>{
            fn set_xy(&mut self, right: &$o<T>){
                self.x = right.x.clone();
                self.y = right.y.clone();
            }

            fn set_yx(&mut self, right: &$o<T>){
                self.y = right.x.clone();
                self.x = right.y.clone();
            }
        }
    )
}

swizzles2_impl!(Point2, Point2);
swizzles2_impl!(Point3, Point2);
swizzles2_impl!(Point4, Point2);
swizzles2_impl!(Point5, Point2);
swizzles2_impl!(Point6, Point2);
swizzles2_impl!(Vector2, Vector2);
swizzles2_impl!(Vector3, Vector2);
swizzles2_impl!(Vector4, Vector2);
swizzles2_impl!(Vector5, Vector2);
swizzles2_impl!(Vector6, Vector2);



// macro_rules! swizzles2_storage_impl{
//     ($dim: ident, $o: ident) => (
//         impl<'a, T, S> Swizzles2<T> for Matrix<T,$dim,U1,S>
//             where T: Scalar,
//                   S: Storage<T,$dim,U1>
//         {
//             type Swizzle2 = $o<T>;
//             fn xy(&self) -> $o<T>{
//                 $o::new(self[0], self[1])
//             }
//             fn yx(&self) -> $o<T>{
//                 $o::new(self[1], self[0])
//             }
//         }

//         impl<'a, T, S> Swizzles2Mut<T> for Matrix<T,$dim,U1,S>
//             where T: Scalar,
//                   S: StorageMut<T,$dim,U1>
//         {
//             fn set_xy(&mut self, right: &$o<T>){
//                 self[0] = right.x;
//                 self[1] = right.y;
//             }

//             fn set_yx(&mut self, right: &$o<T>){
//                 self[0] = right.x;
//                 self[1] = right.y;
//             }
//         }
//     )
// }

// swizzles2_storage_impl!(U2,Vector2);
// swizzles2_storage_impl!(U3,Vector2);
// swizzles2_storage_impl!(U4,Vector2);
// swizzles2_storage_impl!(U5,Vector2);
// swizzles2_storage_impl!(U6,Vector2);

pub trait Swizzles3<T: Scalar>: Swizzles2<T>{
    type Swizzle3;
    fn xyz(&self) -> Self::Swizzle3;
    fn xzy(&self) -> Self::Swizzle3;
    fn yxz(&self) -> Self::Swizzle3;
    fn yzx(&self) -> Self::Swizzle3;
    fn zxy(&self) -> Self::Swizzle3;
    fn zyx(&self) -> Self::Swizzle3;
    fn yz(&self) -> Self::Swizzle2;
    fn xz(&self) -> Self::Swizzle2;
    fn zy(&self) -> Self::Swizzle2;
    fn zx(&self) -> Self::Swizzle2;
}

pub trait Swizzles3Mut<T: Scalar>: Swizzles2Mut<T> + Swizzles3<T>{
    fn set_xyz(&mut self, right: &Self::Swizzle3);
    fn set_xzy(&mut self, right: &Self::Swizzle3);
    fn set_yxz(&mut self, right: &Self::Swizzle3);
    fn set_yzx(&mut self, right: &Self::Swizzle3);
    fn set_zxy(&mut self, right: &Self::Swizzle3);
    fn set_zyx(&mut self, right: &Self::Swizzle3);
    fn set_yz(&mut self, right: &Self::Swizzle2);
    fn set_xz(&mut self, right: &Self::Swizzle2);
    fn set_zy(&mut self, right: &Self::Swizzle2);
    fn set_zx(&mut self, right: &Self::Swizzle2);
}

macro_rules! swizzles3_impl{
    ($v: ident, $o: ident) => (
        impl<T: Scalar> Swizzles3<T> for $v<T>{
            type Swizzle3 = $o<T>;
            fn xyz(&self) -> $o<T>{
                $o::new(self.x.clone(), self.y.clone(), self.z.clone())
            }

            fn xzy(&self) -> $o<T>{
                $o::new(self.x.clone(), self.z.clone(), self.y.clone())
            }

            fn yxz(&self) -> $o<T>{
                $o::new(self.y.clone(), self.x.clone(), self.z.clone())
            }

            fn yzx(&self) -> $o<T>{
                $o::new(self.y.clone(), self.z.clone(), self.x.clone())
            }

            fn zxy(&self) -> $o<T>{
                $o::new(self.z.clone(), self.x.clone(), self.y.clone())
            }

            fn zyx(&self) -> $o<T>{
                $o::new(self.z.clone(), self.y.clone(), self.x.clone())
            }

            fn yz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.y.clone(), self.z.clone())
            }

            fn xz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.x.clone(), self.z.clone())
            }

            fn zy(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.z.clone(), self.y.clone())
            }

            fn zx(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.z.clone(), self.x.clone())
            }
        }

        impl<T: Scalar> Swizzles3Mut<T> for $v<T>{
            fn set_xyz(&mut self, right: &$o<T>){
                self.x = right.x.clone();
                self.y = right.y.clone();
                self.z = right.z.clone();
            }

            fn set_xzy(&mut self, right: &$o<T>){
                self.x = right.x.clone();
                self.z = right.y.clone();
                self.y = right.z.clone();
            }

            fn set_yxz(&mut self, right: &$o<T>){
                self.y = right.x.clone();
                self.x = right.y.clone();
                self.z = right.z.clone();
            }

            fn set_yzx(&mut self, right: &$o<T>){
                self.y = right.x.clone();
                self.z = right.y.clone();
                self.x = right.z.clone();
            }

            fn set_zxy(&mut self, right: &$o<T>){
                self.z = right.x.clone();
                self.x = right.y.clone();
                self.y = right.z.clone();
            }

            fn set_zyx(&mut self, right: &$o<T>){
                self.z = right.x.clone();
                self.y = right.y.clone();
                self.x = right.z.clone();
            }

            fn set_yz(&mut self, right: &Self::Swizzle2){
                self.y = right.x.clone();
                self.z = right.y.clone();
            }

            fn set_xz(&mut self, right: &Self::Swizzle2){
                self.x = right.x.clone();
                self.z = right.y.clone();
            }

            fn set_zy(&mut self, right: &Self::Swizzle2){
                self.z = right.x.clone();
                self.y = right.y.clone();
            }

            fn set_zx(&mut self, right: &Self::Swizzle2){
                self.z = right.x.clone();
                self.x = right.y.clone();
            }
        }
    )
}

swizzles3_impl!(Point3, Point3);
swizzles3_impl!(Point4, Point3);
swizzles3_impl!(Point5, Point3);
swizzles3_impl!(Point6, Point3);
swizzles3_impl!(Vector3, Vector3);
swizzles3_impl!(Vector4, Vector3);
swizzles3_impl!(Vector5, Vector3);
swizzles3_impl!(Vector6, Vector3);





// macro_rules! swizzles3_storage_impl{
//     ($dim: ident, $o: ident) => (
//         impl<'a, T, S> Swizzles3<T> for Matrix<T,$dim,U1,S>
//             where T: Scalar,
//                   S: Storage<T,$dim,U1>
//         {
//             type Swizzle3 = $o<T>;
//             fn xyz(&self) -> $o<T>{
//                 $o::new(self[0], self[1], self[2])
//             }

//             fn xzy(&self) -> $o<T>{
//                 $o::new(self[0], self[2], self[1])
//             }

//             fn yxz(&self) -> $o<T>{
//                 $o::new(self[1], self[0], self[2])
//             }

//             fn yzx(&self) -> $o<T>{
//                 $o::new(self[1], self[2], self[0])
//             }

//             fn zxy(&self) -> $o<T>{
//                 $o::new(self[2], self[0], self[1])
//             }

//             fn zyx(&self) -> $o<T>{
//                 $o::new(self[2], self[1], self[0])
//             }

//             fn yz(&self) -> Self::Swizzle2{
//                 Self::Swizzle2::new(self[1], self[2])
//             }

//             fn xz(&self) -> Self::Swizzle2{
//                 Self::Swizzle2::new(self[0], self[2])
//             }

//             fn zy(&self) -> Self::Swizzle2{
//                 Self::Swizzle2::new(self[2], self[1])
//             }

//             fn zx(&self) -> Self::Swizzle2{
//                 Self::Swizzle2::new(self[2], self[0])
//             }
//         }

//         impl<'a, T, S> Swizzles3Mut<T> for Matrix<T,$dim,U1,S>
//             where T: Scalar,
//                   S: StorageMut<T,$dim,U1>
//         {
//                       fn set_xyz(&mut self, right: &$o<T>){
//                           self[0] = right.x;
//                           self[1] = right.y;
//                           self[2] = right.z;
//                       }

//                       fn set_xzy(&mut self, right: &$o<T>){
//                           self[0] = right.x;
//                           self[2] = right.y;
//                           self[1] = right.z;
//                       }

//                       fn set_yxz(&mut self, right: &$o<T>){
//                           self[1] = right.x;
//                           self[0] = right.y;
//                           self[2] = right.z;
//                       }

//                       fn set_yzx(&mut self, right: &$o<T>){
//                           self[1] = right.x;
//                           self[2] = right.y;
//                           self[0] = right.z;
//                       }

//                       fn set_zxy(&mut self, right: &$o<T>){
//                           self[2] = right.x;
//                           self[0] = right.y;
//                           self[1] = right.z;
//                       }

//                       fn set_zyx(&mut self, right: &$o<T>){
//                           self[2] = right.x;
//                           self[1] = right.y;
//                           self[0] = right.z;
//                       }

//                       fn set_yz(&mut self, right: &Self::Swizzle2){
//                           self[1] = right.x;
//                           self[2] = right.y;
//                       }

//                       fn set_xz(&mut self, right: &Self::Swizzle2){
//                           self[0] = right.x;
//                           self[2] = right.y;
//                       }

//                       fn set_zy(&mut self, right: &Self::Swizzle2){
//                           self[2] = right.x;
//                           self[1] = right.y;
//                       }

//                       fn set_zx(&mut self, right: &Self::Swizzle2){
//                           self[2] = right.x;
//                           self[0] = right.y;
//                       }
//         }
//     )
// }

// swizzles3_storage_impl!(U3,Vector3);
// swizzles3_storage_impl!(U4,Vector3);
// swizzles3_storage_impl!(U5,Vector3);
// swizzles3_storage_impl!(U6,Vector3);



pub trait Swizzles4<T: Scalar>: Swizzles3<T>{
    type Swizzle4;
    fn xyzw(&self) -> Self::Swizzle4;
    fn xyw(&self) -> Self::Swizzle3;
    fn yxw(&self) -> Self::Swizzle3;
    fn wxy(&self) -> Self::Swizzle3;
    fn wyx(&self) -> Self::Swizzle3;
    fn yzw(&self) -> Self::Swizzle3;
    fn zyw(&self) -> Self::Swizzle3;
    fn wyz(&self) -> Self::Swizzle3;
    fn wzy(&self) -> Self::Swizzle3;
    fn xzw(&self) -> Self::Swizzle3;
    fn zxw(&self) -> Self::Swizzle3;
    fn wxz(&self) -> Self::Swizzle3;
    fn wzx(&self) -> Self::Swizzle3;
    fn xw(&self) -> Self::Swizzle2;
    fn yw(&self) -> Self::Swizzle2;
    fn zw(&self) -> Self::Swizzle2;
    fn wx(&self) -> Self::Swizzle2;
    fn wy(&self) -> Self::Swizzle2;
    fn wz(&self) -> Self::Swizzle2;
}

pub trait Swizzles4Mut<T: Scalar>: Swizzles3Mut<T> + Swizzles4<T>{
    fn set_xyzw(&mut self, right: &Self::Swizzle4);
    fn set_xyw(&mut self, right: &Self::Swizzle3);
    fn set_yxw(&mut self, right: &Self::Swizzle3);
    fn set_wxy(&mut self, right: &Self::Swizzle3);
    fn set_wyx(&mut self, right: &Self::Swizzle3);
    fn set_yzw(&mut self, right: &Self::Swizzle3);
    fn set_zyw(&mut self, right: &Self::Swizzle3);
    fn set_wyz(&mut self, right: &Self::Swizzle3);
    fn set_wzy(&mut self, right: &Self::Swizzle3);
    fn set_xzw(&mut self, right: &Self::Swizzle3);
    fn set_zxw(&mut self, right: &Self::Swizzle3);
    fn set_wxz(&mut self, right: &Self::Swizzle3);
    fn set_wzx(&mut self, right: &Self::Swizzle3);
    fn set_xw(&mut self, right: &Self::Swizzle2);
    fn set_yw(&mut self, right: &Self::Swizzle2);
    fn set_zw(&mut self, right: &Self::Swizzle2);
    fn set_wx(&mut self, right: &Self::Swizzle2);
    fn set_wy(&mut self, right: &Self::Swizzle2);
    fn set_wz(&mut self, right: &Self::Swizzle2);
}

macro_rules! swizzles4_impl{
    ($v: ident, $o: ident) => (
        impl<T: Scalar> Swizzles4<T> for $v<T>{
            type Swizzle4 = $o<T>;
            fn xyzw(&self) -> Self::Swizzle4{
                Self::Swizzle4::new(self.x.clone(), self.y.clone(), self.z.clone(), self.w.clone())
            }

            fn xyw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.x.clone(), self.y.clone(), self.w.clone())
            }

            fn yxw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.y.clone(), self.x.clone(), self.w.clone())
            }

            fn wxy(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w.clone(), self.x.clone(), self.y.clone())
            }

            fn wyx(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w.clone(), self.y.clone(), self.x.clone())
            }

            fn yzw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.y.clone(), self.z.clone(), self.w.clone())
            }

            fn zyw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.z.clone(), self.y.clone(), self.w.clone())
            }

            fn wyz(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w.clone(), self.y.clone(), self.z.clone())
            }

            fn wzy(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w.clone(), self.z.clone(), self.y.clone())
            }

            fn xzw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.x.clone(), self.z.clone(), self.w.clone())
            }

            fn zxw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.z.clone(), self.x.clone(), self.w.clone())
            }

            fn wxz(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w.clone(), self.x.clone(), self.x.clone())
            }

            fn wzx(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w.clone(), self.z.clone(), self.x.clone())
            }

            fn xw(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.x.clone(), self.w.clone())
            }

            fn yw(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.y.clone(), self.w.clone())
            }

            fn zw(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.z.clone(), self.w.clone())
            }

            fn wx(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.w.clone(), self.x.clone())
            }

            fn wy(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.w.clone(), self.y.clone())
            }

            fn wz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.w.clone(), self.z.clone())
            }

        }

        impl<T: Scalar> Swizzles4Mut<T> for $v<T>{
            fn set_xyzw(&mut self, right: &Self::Swizzle4) {
                self.x = right.x.clone();
                self.y = right.y.clone();
                self.z = right.z.clone();
                self.w = right.w.clone();
            }

            fn set_xyw(&mut self, right: &Self::Swizzle3) {
                self.x = right.x.clone();
                self.y = right.y.clone();
                self.w = right.z.clone();
            }

            fn set_yxw(&mut self, right: &Self::Swizzle3) {
                self.y = right.x.clone();
                self.x = right.y.clone();
                self.w = right.z.clone();
            }

            fn set_wxy(&mut self, right: &Self::Swizzle3) {
                self.w = right.x.clone();
                self.x = right.y.clone();
                self.y = right.z.clone();
            }

            fn set_wyx(&mut self, right: &Self::Swizzle3) {
                self.w = right.x.clone();
                self.y = right.y.clone();
                self.x = right.z.clone();
            }

            fn set_yzw(&mut self, right: &Self::Swizzle3) {
                self.y = right.x.clone();
                self.z = right.y.clone();
                self.w = right.z.clone();
            }

            fn set_zyw(&mut self, right: &Self::Swizzle3) {
                self.z = right.x.clone();
                self.y = right.y.clone();
                self.w = right.z.clone();
            }

            fn set_wyz(&mut self, right: &Self::Swizzle3) {
                self.w = right.x.clone();
                self.y = right.y.clone();
                self.z = right.z.clone();
            }

            fn set_wzy(&mut self, right: &Self::Swizzle3) {
                self.w = right.x.clone();
                self.z = right.y.clone();
                self.y = right.z.clone();
            }

            fn set_xzw(&mut self, right: &Self::Swizzle3) {
                self.x = right.x.clone();
                self.z = right.y.clone();
                self.w = right.z.clone();
            }

            fn set_zxw(&mut self, right: &Self::Swizzle3) {
                self.z = right.x.clone();
                self.x = right.y.clone();
                self.w = right.z.clone();
            }

            fn set_wxz(&mut self, right: &Self::Swizzle3) {
                self.w = right.x.clone();
                self.x = right.y.clone();
                self.z = right.z.clone();
            }

            fn set_wzx(&mut self, right: &Self::Swizzle3) {
                self.w = right.x.clone();
                self.z = right.y.clone();
                self.x = right.z.clone();
            }

            fn set_xw(&mut self, right: &Self::Swizzle2) {
                self.x = right.x.clone();
                self.w = right.y.clone();
            }

            fn set_yw(&mut self, right: &Self::Swizzle2) {
                self.y = right.x.clone();
                self.w = right.y.clone();
            }

            fn set_zw(&mut self, right: &Self::Swizzle2) {
                self.z = right.x.clone();
                self.w = right.y.clone();
            }

            fn set_wx(&mut self, right: &Self::Swizzle2) {
                self.w = right.x.clone();
                self.x = right.y.clone();
            }

            fn set_wy(&mut self, right: &Self::Swizzle2) {
                self.w = right.x.clone();
                self.y = right.y.clone();
            }

            fn set_wz(&mut self, right: &Self::Swizzle2) {
                self.w = right.x.clone();
                self.z = right.y.clone();
            }

        }
    )
}

swizzles4_impl!(Vector4, Vector4);
swizzles4_impl!(Vector5, Vector4);
swizzles4_impl!(Vector6, Vector4);
swizzles4_impl!(Point4, Point4);
swizzles4_impl!(Point5, Point4);
swizzles4_impl!(Point6, Point4);
