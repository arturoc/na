extern crate nalgebra as na;
extern crate alga;
extern crate num;
extern crate typenum;
extern crate generic_array;

pub type VecN<T,D> = na::VectorN<T,D>;
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

pub type Ortho3<T=f32> = na::Orthographic3<T>;
pub type Persp3<T=f32> = na::Perspective3<T>;

pub type Rot2<T=f32> = na::Rotation2<T>;
pub type Rot3<T=f32> = na::Rotation3<T>;

pub type Quat<T=f32> = na::Quaternion<T>;

pub type Iso2<T=f32> = na::Isometry2<T>;
pub type Iso3<T=f32> = na::Isometry3<T>;

pub type DVec<T=f32> = na::DVector<T>;

pub type DMat<T=f32> = na::DMatrix<T>;

pub type UnitQuat<T=f32> = na::UnitQuaternion<T>;

pub use na::*;
pub use std::mem;
pub use std::ops::*;
pub use std::fmt::Debug;

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
        impl<T: alga::general::Real> ToPnt<$p<T>> for $v<T>{
            #[inline]
            fn to_pnt(self) -> $p<T>{
                $p::from_coordinates(self)
            }
        }

        impl<T: alga::general::Real> AsPnt<$p<T>> for $v<T>{
            #[inline]
            fn as_pnt(&self) -> &$p<T>{
                unsafe{ mem::transmute(self) }
            }
        }

        impl<T: alga::general::Real> ToVec<$v<T>> for $p<T>{
            #[inline]
            fn to_vec(self) -> $v<T>{
                self.coords
            }
        }

        impl<T: alga::general::Real> AsVec<$v<T>> for $p<T>{
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


impl<T: alga::general::Real> ToMat<Matrix4<T>> for Orthographic3<T>{
    fn to_mat(self) -> Matrix4<T>{
        self.to_homogeneous()
    }
}

impl<T: alga::general::Real> AsMat<Matrix4<T>> for Orthographic3<T>{
    fn as_mat(&self) -> &Matrix4<T>{
        self.as_matrix()
    }
}

pub trait FastDot<T>{
    fn fast_dot(&self, right: &Self) -> T;
}

impl<T:alga::general::Real> FastDot<T> for [T;2]{
    #[inline]
    fn fast_dot(&self, right: &[T;2]) -> T{
        self[0] * right[0] + self[1] * right[1]
    }
}

impl<T:alga::general::Real> FastDot<T> for [T;3]{
    #[inline]
    fn fast_dot(&self, right: &[T;3]) -> T{
        self[0] * right[0] + self[1] * right[1] + self[2] * right[2]
    }
}

impl<T:alga::general::Real> FastDot<T> for [T;4]{
    #[inline]
    fn fast_dot(&self, right: &[T;4]) -> T{
        self[0] * right[0] + self[1] * right[1] + self[2] * right[2] + self[3] * right[3]
    }
}

impl<T:alga::general::Real> FastDot<T> for [T;5]{
    #[inline]
    fn fast_dot(&self, right: &[T;5]) -> T{
        self[0] * right[0] + self[1] * right[1] + self[2] * right[2] + self[3] * right[3] + self[4] * right[4]
    }
}

impl<T:alga::general::Real> FastDot<T> for [T;6]{
    #[inline]
    fn fast_dot(&self, right: &[T;6]) -> T{
        self[0] * right[0] + self[1] * right[1] + self[2] * right[2] + self[3] * right[3] + self[4] * right[4] + self[5] * right[5]
    }
}

impl<T:alga::general::Real> FastDot<T> for Vector2<T>{
    #[inline]
    fn fast_dot(&self, right: &Vector2<T>) -> T{
        self.x * right.x + self.y * right.y
    }
}

impl<T:alga::general::Real> FastDot<T> for Vector3<T>{
    #[inline]
    fn fast_dot(&self, right: &Vector3<T>) -> T{
        self.x * right.x + self.y * right.y + self.z * right.z
    }
}

impl<T:alga::general::Real> FastDot<T> for Vector4<T>{
    #[inline]
    fn fast_dot(&self, right: &Vector4<T>) -> T{
        self.x * right.x + self.y * right.y + self.z * right.z + self.w * right.w
    }
}

impl<T:alga::general::Real> FastDot<T> for Vector5<T>{
    #[inline]
    fn fast_dot(&self, right: &Vector5<T>) -> T{
        self.x * right.x + self.y * right.y + self.z * right.z + self.w * right.w + self.a * right.a
    }
}

impl<T:alga::general::Real> FastDot<T> for Vector6<T>{
    #[inline]
    fn fast_dot(&self, right: &Vector6<T>) -> T{
        self.x * right.x + self.y * right.y + self.z * right.z + self.w * right.w + self.a * right.a + self.b * right.b
    }
}



pub trait FastMul<T>{
    type Output;
    fn fast_mul(&self, right: &T) -> Self::Output;
}

impl<T:alga::general::Real> FastMul<Matrix2<T>> for Matrix2<T>{
    type Output = Matrix2<T>;
    #[inline]
    fn fast_mul(&self, right: &Matrix2<T>) -> Matrix2<T>{
        let row0 = [self.m11, self.m12];
        let row1 = [self.m21, self.m22];
        let col0 = [right.m11, right.m21];
        let col1 = [right.m12, right.m22];
        Matrix2::new(
            row0.fast_dot(&col0), row0.fast_dot(&col1),
            row1.fast_dot(&col0), row1.fast_dot(&col1),
        )
    }
}

impl<T:alga::general::Real> FastMul<Vector2<T>> for Matrix2<T>{
    type Output = Vector2<T>;
    #[inline]
    fn fast_mul(&self, right: &Vector2<T>) -> Vector2<T>{
        let row0 = [self.m11, self.m12];
        let row1 = [self.m21, self.m22];
        let right = right.as_ref();
        Vector2::new(
            row0.fast_dot(right), row1.fast_dot(right)
        )
    }
}

impl<T:alga::general::Real> FastMul<Matrix3<T>> for Matrix3<T>{
    type Output = Matrix3<T>;
    #[inline]
    fn fast_mul(&self, right: &Matrix3<T>) -> Matrix3<T>{
        let row0 = [self.m11, self.m12, self.m13];
        let row1 = [self.m21, self.m22, self.m23];
        let row2 = [self.m31, self.m32, self.m33];
        let col0 = [right.m11, right.m21, right.m31];
        let col1 = [right.m12, right.m22, right.m32];
        let col2 = [right.m13, right.m23, right.m33];
        Matrix3::new(
            row0.fast_dot(&col0), row0.fast_dot(&col1), row0.fast_dot(&col2),
            row1.fast_dot(&col0), row1.fast_dot(&col1), row1.fast_dot(&col2),
            row2.fast_dot(&col0), row2.fast_dot(&col1), row2.fast_dot(&col2),
        )
    }
}

impl<T:alga::general::Real> FastMul<Vector3<T>> for Matrix3<T>{
    type Output = Vector3<T>;
    #[inline]
    fn fast_mul(&self, right: &Vector3<T>) -> Vector3<T>{
        let row0 = [self.m11, self.m12, self.m13];
        let row1 = [self.m21, self.m22, self.m23];
        let row2 = [self.m31, self.m32, self.m33];
        let right = right.as_ref();
        Vector3::new(
            row0.fast_dot(right), row1.fast_dot(right), row2.fast_dot(right)
        )
    }
}

impl<T:alga::general::Real> FastMul<Matrix4<T>> for Matrix4<T>{
    type Output = Matrix4<T>;
    #[inline]
    fn fast_mul(&self, right: &Matrix4<T>) -> Matrix4<T>{
        let row0 = [self.m11, self.m12, self.m13, self.m14];
        let row1 = [self.m21, self.m22, self.m23, self.m24];
        let row2 = [self.m31, self.m32, self.m33, self.m34];
        let row3 = [self.m41, self.m42, self.m43, self.m44];
        let col0 = [right.m11, right.m21, right.m31, right.m41];
        let col1 = [right.m12, right.m22, right.m32, right.m42];
        let col2 = [right.m13, right.m23, right.m33, right.m43];
        let col3 = [right.m14, right.m24, right.m34, right.m44];
        Matrix4::new(
            row0.fast_dot(&col0), row0.fast_dot(&col1), row0.fast_dot(&col2), row0.fast_dot(&col3),
            row1.fast_dot(&col0), row1.fast_dot(&col1), row1.fast_dot(&col2), row1.fast_dot(&col3),
            row2.fast_dot(&col0), row2.fast_dot(&col1), row2.fast_dot(&col2), row2.fast_dot(&col3),
            row3.fast_dot(&col0), row3.fast_dot(&col1), row3.fast_dot(&col2), row3.fast_dot(&col3),
        )
    }
}

impl<T:alga::general::Real> FastMul<Vector4<T>> for Matrix4<T>{
    type Output = Vector4<T>;
    #[inline]
    fn fast_mul(&self, right: &Vector4<T>) -> Vector4<T>{
        let row0 = [self.m11, self.m12, self.m13, self.m14];
        let row1 = [self.m21, self.m22, self.m23, self.m24];
        let row2 = [self.m31, self.m32, self.m33, self.m34];
        let row3 = [self.m41, self.m42, self.m43, self.m44];
        let right = right.as_ref();
        Vector4::new(
            row0.fast_dot(right), row1.fast_dot(right), row2.fast_dot(right), row3.fast_dot(right)
        )
    }
}

pub trait FastInverse{
    fn fast_orthonormal_inverse(&self) -> Self;
    fn fast_affine_inverse(&self) -> Self;
}

impl<T:alga::general::Real> FastInverse for Matrix4<T>{
    fn fast_orthonormal_inverse(&self) -> Matrix4<T>{
        let _3x3 = Matrix3::new(
            self.m11, self.m21, self.m31,
            self.m12, self.m22, self.m32,
            self.m13, self.m23, self.m33,
        );
        let pos = vec3(self.m14, self.m24, self.m34);
        let pos = -_3x3.fast_mul(&pos);
        Matrix4::new(
            self.m11, self.m21, self.m31, pos.x,
            self.m12, self.m22, self.m32, pos.y,
            self.m13, self.m23, self.m33, pos.z,
            zero(),   zero(),   zero(),   one()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ,
        )
    }

    fn fast_affine_inverse(&self) -> Matrix4<T>{
        let _3x3 = Matrix3::new(
            self.m11, self.m12, self.m13,
            self.m21, self.m22, self.m23,
            self.m31, self.m32, self.m33,
        ).try_inverse().unwrap();

        let pos = vec3(self.m14, self.m24, self.m34);
        let pos = -_3x3.fast_mul(&pos);
        Matrix4::new(
            _3x3.m11, _3x3.m12, _3x3.m13, pos.x,
            _3x3.m21, _3x3.m22, _3x3.m23, pos.y,
            _3x3.m31, _3x3.m32, _3x3.m33, pos.z,
            zero(),   zero(),   zero(),   one()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ,
        )
    }
}

pub trait BaseNum: Scalar +
                   alga::general::Identity<alga::general::Additive> +
                   alga::general::Identity<alga::general::Multiplicative> +
                   num::Zero +
                   num::One +
                   Add<Self, Output = Self> + Sub<Self, Output = Self> +
                   Mul<Self, Output = Self> + Div<Self, Output = Self> +
                   Rem<Self, Output = Self> +
                   AddAssign<Self> + SubAssign<Self> +
                   MulAssign<Self> + DivAssign<Self> +
                   RemAssign<Self> +
                   PartialOrd +
                   'static{
}

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

pub trait BaseInt: BaseNum +
                   Shl<Self, Output = Self> +
                   ShlAssign<Self> +
                   Shr<Self, Output=Self> +
                   ShrAssign<Self>{}

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
pub trait NumVec<N>: Add<Self, Output = Self> + Sub<Self, Output = Self> +
                        // Mul<Self, Output = Self> + Div<Self, Output = Self> +

                        // Add<N, Output = Self> + Sub<N, Output = Self> +
                        Mul<N, Output = Self> + Div<N, Output = Self> +

                        AddAssign<Self> + SubAssign<Self> +
                        // MulAssign<Self> + DivAssign<Self> +

                        // AddAssign<N> + SubAssign<N> +
                        MulAssign<N> + DivAssign<N> +

                        alga::general::Identity<alga::general::Additive> +
                        PartialEq
                        where Self: Sized {
}

/// Trait of vector with components implementing the `BaseFloat` trait.
pub trait FloatVec<N: alga::general::Real>: NumVec<N> +
                        alga::linear::NormedSpace +
                        Neg<Output = Self> +
                        Axpy<N> +
                        alga::linear::FiniteDimVectorSpace +
                        alga::linear::FiniteDimInnerSpace {
}


impl<N: BaseNum> NumVec<N> for Vector1<N>{}
impl<N: BaseNum> NumVec<N> for Vector2<N>{}
impl<N: BaseNum> NumVec<N> for Vector3<N>{}
impl<N: BaseNum> NumVec<N> for Vector4<N>{}
impl<N: BaseNum> NumVec<N> for Vector5<N>{}
impl<N: BaseNum> NumVec<N> for Vector6<N>{}

impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector1<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector2<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector3<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector4<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector5<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector6<N>{}

/*
 * Vector related traits.
 */
/// Trait grouping most common operations on vectors.
pub trait NumPnt<N>: // Sub<Self, Output = NumVec<N>> + //TODO: Output Vec
                        // Mul<Self, Output = Self> + Div<Self, Output = Self> +

                        // Add<N, Output = Self> + Sub<N, Output = Self> +
                        Mul<N, Output = Self> + Div<N, Output = Self> +

                        // MulAssign<Self> + DivAssign<Self> +

                        // AddAssign<N> + SubAssign<N> +
                        MulAssign<N> + DivAssign<N> +
                        PartialEq
                        where Self: Sized {
}

/// Trait of vector with components implementing the `BaseFloat` trait.
pub trait FloatPnt<N: alga::general::Real>: NumPnt<N> +
                        Neg<Output = Self> +
                        Axpy<N> +
                        alga::linear::AffineSpace +
                        alga::linear::EuclideanSpace {
}


impl<N: BaseNum> NumPnt<N> for Point1<N>{}
impl<N: BaseNum> NumPnt<N> for Point2<N>{}
impl<N: BaseNum> NumPnt<N> for Point3<N>{}
impl<N: BaseNum> NumPnt<N> for Point4<N>{}
impl<N: BaseNum> NumPnt<N> for Point5<N>{}
impl<N: BaseNum> NumPnt<N> for Point6<N>{}

impl<N: BaseNum + alga::general::Real> FloatPnt<N> for Point1<N>{}
impl<N: BaseNum + alga::general::Real> FloatPnt<N> for Point2<N>{}
impl<N: BaseNum + alga::general::Real> FloatPnt<N> for Point3<N>{}
impl<N: BaseNum + alga::general::Real> FloatPnt<N> for Point4<N>{}
impl<N: BaseNum + alga::general::Real> FloatPnt<N> for Point5<N>{}
impl<N: BaseNum + alga::general::Real> FloatPnt<N> for Point6<N>{}


#[macro_export]
macro_rules! vec2{
    ($x: expr, $y: expr) => ({
        Vector2::new($x, $y)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoVec;
        let v: Vector2<_> = $v1.into_vec();
        v
    });
}

#[macro_export]
macro_rules! vec3{
    ($x: expr, $y: expr, $z: expr) => ({
        Vector3::new($x, $y, $z)
    });
    ($v1: expr, $v2: expr) => ({
        use $crate::traits::JoinVec;
        $v1.join($v2)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoVec;
        let v: Vector3<_> = $v1.into_vec();
        v
    });
}

#[macro_export]
macro_rules! vec4{
    ($x: expr, $y: expr, $z: expr, $w: expr) => ({
        Vector4::new($x, $y, $z, $w)
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
        let v: Vector4<_> = $v1.into_vec();
        v
    });
}


#[macro_export]
macro_rules! pnt2{
    ($x: expr, $y: expr) => ({
        Point2::new($x, $y)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoPnt;
        let v: Point2<_> = $v1.into_vec();
        v
    });
}

#[macro_export]
macro_rules! pnt3{
    ($x: expr, $y: expr, $z: expr) => ({
        Point3::new($x, $y, $z)
    });
    ($v1: expr, $v2: expr) => ({
        use $crate::traits::JoinPnt;
        $v1.join($v2)
    });
    ($v1: expr) => ({
        use $crate::traits::IntoPnt;
        let v: Point3<_> = $v1.into_vec();
        v
    });
}

#[macro_export]
macro_rules! pnt4{
    ($x: expr, $y: expr, $z: expr, $w: expr) => ({
        Point4::new($x, $y, $z, $w)
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
        let v: Vector4<_> = $v1.into_vec();
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
                $o::new(self.x, self.y)
            }
            fn yx(&self) -> $o<T>{
                $o::new(self.y, self.x)
            }
        }

        impl<T: Scalar> Swizzles2Mut<T> for $v<T>{
            fn set_xy(&mut self, right: &$o<T>){
                self.x = right.x;
                self.y = right.y;
            }

            fn set_yx(&mut self, right: &$o<T>){
                self.y = right.x;
                self.x = right.y;
            }
        }
    )
}

swizzles2_impl!(Vector2, Vector2);
swizzles2_impl!(Vector3, Vector2);
swizzles2_impl!(Vector4, Vector2);
swizzles2_impl!(Vector5, Vector2);
swizzles2_impl!(Vector6, Vector2);

swizzles2_impl!(Point2, Point2);
swizzles2_impl!(Point3, Point2);
swizzles2_impl!(Point4, Point2);
swizzles2_impl!(Point5, Point2);
swizzles2_impl!(Point6, Point2);



macro_rules! swizzles2_slice_impl{
    ($dim: ident, $o: ident) => (
        impl<'a, T, RStride, CStride, Alloc> Swizzles2<T> for Matrix<T,$dim,U1,SliceStorage<'a,T,$dim,U1,RStride,CStride,Alloc>>
            where T: Scalar,
                  RStride:Dim,
                  CStride:Dim,
                  Alloc: allocator::Allocator<T,$dim,U1> {
            type Swizzle2 = $o<T>;
            fn xy(&self) -> $o<T>{
                $o::new(self[0], self[1])
            }
            fn yx(&self) -> $o<T>{
                $o::new(self[1], self[0])
            }
        }

        impl<'a, T, RStride, CStride, Alloc> Swizzles2<T> for Matrix<T,$dim,U1,SliceStorageMut<'a,T,$dim,U1,RStride,CStride,Alloc>>
            where T: Scalar,
                  RStride:Dim,
                  CStride:Dim,
                  Alloc: allocator::Allocator<T,$dim,U1> {
            type Swizzle2 = $o<T>;
            fn xy(&self) -> $o<T>{
                $o::new(self[0], self[1])
            }
            fn yx(&self) -> $o<T>{
                $o::new(self[1], self[0])
            }
        }

        impl<'a, T, RStride, CStride, Alloc> Swizzles2Mut<T> for Matrix<T,$dim,U1,SliceStorageMut<'a,T,$dim,U1,RStride,CStride,Alloc>>
            where T: Scalar,
                  RStride:Dim,
                  CStride:Dim,
                  Alloc: allocator::Allocator<T,$dim,U1> {
            fn set_xy(&mut self, right: &$o<T>){
                self[0] = right.x;
                self[1] = right.y;
            }

            fn set_yx(&mut self, right: &$o<T>){
                self[0] = right.x;
                self[1] = right.y;
            }
        }
    )
}

swizzles2_slice_impl!(U2,Vector2);
swizzles2_slice_impl!(U3,Vector2);
swizzles2_slice_impl!(U4,Vector2);
swizzles2_slice_impl!(U5,Vector2);
swizzles2_slice_impl!(U6,Vector2);

pub trait Swizzles3<T: Scalar>: Swizzles2<T>{
    type Swizzle3;
    fn xyz(&self) -> Self::Swizzle3;
    fn yz(&self) -> Self::Swizzle2;
    fn xz(&self) -> Self::Swizzle2;
    fn zy(&self) -> Self::Swizzle2;
    fn zx(&self) -> Self::Swizzle2;
}

pub trait Swizzles3Mut<T: Scalar>: Swizzles2Mut<T> + Swizzles3<T>{
    fn set_xyz(&mut self, right: &Self::Swizzle3);
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
                $o::new(self.x, self.y, self.z)
            }

            fn yz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.y, self.z)
            }

            fn xz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.x, self.z)
            }

            fn zy(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.z, self.y)
            }

            fn zx(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.z, self.x)
            }
        }

        impl<T: Scalar> Swizzles3Mut<T> for $v<T>{
            fn set_xyz(&mut self, right: &$o<T>){
                self.x = right.x;
                self.y = right.y;
                self.z = right.z;
            }

            fn set_yz(&mut self, right: &Self::Swizzle2){
                self.y = right.x;
                self.z = right.y;
            }

            fn set_xz(&mut self, right: &Self::Swizzle2){
                self.x = right.x;
                self.z = right.y;
            }

            fn set_zy(&mut self, right: &Self::Swizzle2){
                self.z = right.x;
                self.y = right.y;
            }

            fn set_zx(&mut self, right: &Self::Swizzle2){
                self.z = right.x;
                self.x = right.y;
            }
        }
    )
}

swizzles3_impl!(Vector3, Vector3);
swizzles3_impl!(Vector4, Vector3);
swizzles3_impl!(Vector5, Vector3);
swizzles3_impl!(Vector6, Vector3);

swizzles3_impl!(Point3, Point3);
swizzles3_impl!(Point4, Point3);
swizzles3_impl!(Point5, Point3);
swizzles3_impl!(Point6, Point3);





macro_rules! swizzles3_slice_impl{
    ($dim: ident, $o: ident) => (
        impl<'a, T, RStride, CStride, Alloc> Swizzles3<T> for Matrix<T,$dim,U1,SliceStorage<'a,T,$dim,U1,RStride,CStride,Alloc>>
            where T: Scalar,
                  RStride:Dim,
                  CStride:Dim,
                  Alloc: allocator::Allocator<T,$dim,U1> {
            type Swizzle3 = $o<T>;
            fn xyz(&self) -> $o<T>{
                $o::new(self[0], self[1], self[2])
            }

            fn yz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self[1], self[2])
            }

            fn xz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self[0], self[2])
            }

            fn zy(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self[2], self[1])
            }

            fn zx(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self[2], self[0])
            }
        }

        impl<'a, T, RStride, CStride, Alloc> Swizzles3<T> for Matrix<T,$dim,U1,SliceStorageMut<'a,T,$dim,U1,RStride,CStride,Alloc>>
            where T: Scalar,
                  RStride:Dim,
                  CStride:Dim,
                  Alloc: allocator::Allocator<T,$dim,U1> {
            type Swizzle3 = $o<T>;
            fn xyz(&self) -> $o<T>{
                $o::new(self[0], self[1], self[2])
            }

            fn yz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self[1], self[2])
            }

            fn xz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self[0], self[2])
            }

            fn zy(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self[2], self[1])
            }

            fn zx(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self[2], self[0])
            }
        }

        impl<'a, T, RStride, CStride, Alloc> Swizzles3Mut<T> for Matrix<T,$dim,U1,SliceStorageMut<'a,T,$dim,U1,RStride,CStride,Alloc>>
            where T: Scalar,
                  RStride:Dim,
                  CStride:Dim,
                  Alloc: allocator::Allocator<T,$dim,U1> {
                      fn set_xyz(&mut self, right: &$o<T>){
                          self[0] = right.x;
                          self[1] = right.y;
                          self[2] = right.z;
                      }

                      fn set_yz(&mut self, right: &Self::Swizzle2){
                          self[1] = right.x;
                          self[2] = right.y;
                      }

                      fn set_xz(&mut self, right: &Self::Swizzle2){
                          self[0] = right.x;
                          self[2] = right.y;
                      }

                      fn set_zy(&mut self, right: &Self::Swizzle2){
                          self[2] = right.x;
                          self[1] = right.y;
                      }

                      fn set_zx(&mut self, right: &Self::Swizzle2){
                          self[2] = right.x;
                          self[0] = right.y;
                      }
        }
    )
}

swizzles3_slice_impl!(U3,Vector3);
swizzles3_slice_impl!(U4,Vector3);
swizzles3_slice_impl!(U5,Vector3);
swizzles3_slice_impl!(U6,Vector3);



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
                Self::Swizzle4::new(self.x, self.y, self.z, self.w)
            }

            fn xyw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.x, self.y, self.w)
            }

            fn yxw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.y, self.x, self.w)
            }

            fn wxy(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w, self.x, self.y)
            }

            fn wyx(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w, self.y, self.x)
            }

            fn yzw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.y, self.z, self.w)
            }

            fn zyw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.z, self.y, self.w)
            }

            fn wyz(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w, self.y, self.z)
            }

            fn wzy(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w, self.z, self.y)
            }

            fn xzw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.x, self.z, self.w)
            }

            fn zxw(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.z, self.x, self.w)
            }

            fn wxz(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w, self.x, self.x)
            }

            fn wzx(&self) -> Self::Swizzle3{
                Self::Swizzle3::new(self.w, self.z, self.x)
            }

            fn xw(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.x, self.w)
            }

            fn yw(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.y, self.w)
            }

            fn zw(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.z, self.w)
            }

            fn wx(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.w, self.x)
            }

            fn wy(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.w, self.y)
            }

            fn wz(&self) -> Self::Swizzle2{
                Self::Swizzle2::new(self.w, self.z)
            }

        }

        impl<T: Scalar> Swizzles4Mut<T> for $v<T>{
            fn set_xyzw(&mut self, right: &Self::Swizzle4) {
                self.x = right.x;
                self.y = right.y;
                self.z = right.z;
                self.w = right.w;
            }

            fn set_xyw(&mut self, right: &Self::Swizzle3) {
                self.x = right.x;
                self.y = right.y;
                self.w = right.z;
            }

            fn set_yxw(&mut self, right: &Self::Swizzle3) {
                self.y = right.x;
                self.x = right.y;
                self.w = right.z;
            }

            fn set_wxy(&mut self, right: &Self::Swizzle3) {
                self.w = right.x;
                self.x = right.y;
                self.y = right.z;
            }

            fn set_wyx(&mut self, right: &Self::Swizzle3) {
                self.w = right.x;
                self.y = right.y;
                self.x = right.z;
            }

            fn set_yzw(&mut self, right: &Self::Swizzle3) {
                self.y = right.x;
                self.z = right.y;
                self.w = right.z;
            }

            fn set_zyw(&mut self, right: &Self::Swizzle3) {
                self.z = right.x;
                self.y = right.y;
                self.w = right.z;
            }

            fn set_wyz(&mut self, right: &Self::Swizzle3) {
                self.w = right.x;
                self.y = right.y;
                self.z = right.z;
            }

            fn set_wzy(&mut self, right: &Self::Swizzle3) {
                self.w = right.x;
                self.z = right.y;
                self.y = right.z;
            }

            fn set_xzw(&mut self, right: &Self::Swizzle3) {
                self.x = right.x;
                self.z = right.y;
                self.w = right.z;
            }

            fn set_zxw(&mut self, right: &Self::Swizzle3) {
                self.z = right.x;
                self.x = right.y;
                self.w = right.z;
            }

            fn set_wxz(&mut self, right: &Self::Swizzle3) {
                self.w = right.x;
                self.x = right.y;
                self.z = right.z;
            }

            fn set_wzx(&mut self, right: &Self::Swizzle3) {
                self.w = right.x;
                self.z = right.y;
                self.x = right.z;
            }

            fn set_xw(&mut self, right: &Self::Swizzle2) {
                self.x = right.x;
                self.w = right.y;
            }

            fn set_yw(&mut self, right: &Self::Swizzle2) {
                self.y = right.x;
                self.w = right.y;
            }

            fn set_zw(&mut self, right: &Self::Swizzle2) {
                self.z = right.x;
                self.w = right.y;
            }

            fn set_wx(&mut self, right: &Self::Swizzle2) {
                self.w = right.x;
                self.x = right.y;
            }

            fn set_wy(&mut self, right: &Self::Swizzle2) {
                self.w = right.x;
                self.y = right.y;
            }

            fn set_wz(&mut self, right: &Self::Swizzle2) {
                self.w = right.x;
                self.z = right.y;
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
