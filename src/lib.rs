extern crate nalgebra as na;
extern crate alga;
extern crate num;

pub use na::VectorN as VecN;
pub use na::Vector1 as Vec1;
pub use na::Vector2 as Vec2;
pub use na::Vector3 as Vec3;
pub use na::Vector4 as Vec4;
pub use na::Vector5 as Vec5;
pub use na::Vector6 as Vec6;

pub use na::Point1 as Pnt1;
pub use na::Point2 as Pnt2;
pub use na::Point3 as Pnt3;
pub use na::Point4 as Pnt4;
pub use na::Point5 as Pnt5;
pub use na::Point6 as Pnt6;

pub use na::Matrix1 as Mat1;
pub use na::Matrix2 as Mat2;
pub use na::Matrix3 as Mat3;
pub use na::Matrix4 as Mat4;
pub use na::Matrix5 as Mat5;
pub use na::Matrix6 as Mat6;

pub use na::Orthographic3 as OrthoMat3;
pub use na::Perspective3 as PerspMat3;

pub use na::Rotation2 as Rot2;
pub use na::Rotation3 as Rot3;

pub use na::Quaternion as Quat;

pub use na::Isometry2 as Iso2;
pub use na::Isometry3 as Iso3;

pub use na::DVector as DVec;

pub use na::DMatrix as DMat;

pub use na::UnitQuaternion as UnitQuat;

pub use na::*;
pub use std::mem;
pub use std::ops::*;
pub use std::fmt::Debug;

// vec constructors
#[inline]
pub fn vec2<T: Copy + PartialEq + Debug + 'static>(x: T, y: T) -> Vec2<T>{
    Vec2::new(x,y)
}

#[inline]
pub fn vec3<T: Copy + PartialEq + Debug + 'static>(x: T, y: T, z: T) -> Vec3<T>{
    Vec3::new(x,y,z)
}

#[inline]
pub fn vec4<T: Copy + PartialEq + Debug + 'static>(x: T, y: T, z: T, w: T) -> Vec4<T>{
    Vec4::new(x,y,z,w)
}

#[inline]
pub fn vec5<T: Copy + PartialEq + Debug + 'static>(x: T, y: T, z: T, w: T, a: T) -> Vec5<T>{
    Vec5::new(x,y,z,w,a)
}

#[inline]
pub fn vec6<T: Copy + PartialEq + Debug + 'static>(x: T, y: T, z: T, w: T, a: T, b: T) -> Vec6<T>{
    Vec6::new(x,y,z,w,a,b)
}

#[inline]
pub fn pnt2<T: Copy + PartialEq + Debug + 'static>(x: T, y: T) -> Pnt2<T>{
    Pnt2::new(x,y)
}

#[inline]
pub fn pnt3<T: Copy + PartialEq + Debug + 'static>(x: T, y: T, z: T) -> Pnt3<T>{
    Pnt3::new(x,y,z)
}

#[inline]
pub fn pnt4<T: Copy + PartialEq + Debug + 'static>(x: T, y: T, z: T, w: T) -> Pnt4<T>{
    Pnt4::new(x,y,z,w)
}

#[inline]
pub fn pnt5<T: Copy + PartialEq + Debug + 'static>(x: T, y: T, z: T, w: T, a: T) -> Pnt5<T>{
    Pnt5::new(x,y,z,w,a)
}

#[inline]
pub fn pnt6<T: Copy + PartialEq + Debug + 'static>(x: T, y: T, z: T, w: T, a: T, b: T) -> Pnt6<T>{
    Pnt6::new(x,y,z,w,a,b)
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

vec_to_pnt_impl!(Vec1, Pnt1);
vec_to_pnt_impl!(Vec2, Pnt2);
vec_to_pnt_impl!(Vec3, Pnt3);
vec_to_pnt_impl!(Vec4, Pnt4);
vec_to_pnt_impl!(Vec5, Pnt5);
vec_to_pnt_impl!(Vec6, Pnt6);

pub trait ToMat<T>{
    fn to_mat(self) -> T;
}

pub trait AsMat<T>{
    fn as_mat(&self) -> &T;
}


impl<T: alga::general::Real> ToMat<Mat4<T>> for OrthoMat3<T>{
    fn to_mat(self) -> Mat4<T>{
        self.to_homogeneous()
    }
}

impl<T: alga::general::Real> AsMat<Mat4<T>> for OrthoMat3<T>{
    fn as_mat(&self) -> &Mat4<T>{
        self.as_matrix()
    }
}

pub trait FastDot<T>{
    fn fast_dot(&self, right: &Self) -> T;
}

impl<T:alga::general::Real> FastDot<T> for [T;4]{
    #[inline]
    fn fast_dot(&self, right: &[T;4]) -> T{
        self[0] * right[0] + self[1] * right[1] + self[2] * right[2] + self[3] * right[3]
    }
}

impl<T:alga::general::Real> FastDot<T> for [T;3]{
    #[inline]
    fn fast_dot(&self, right: &[T;3]) -> T{
        self[0] * right[0] + self[1] * right[1] + self[2] * right[2]
    }
}

impl<T:alga::general::Real> FastDot<T> for Vec4<T>{
    #[inline]
    fn fast_dot(&self, right: &Vec4<T>) -> T{
        self.x * right.x + self.y * right.y + self.z * right.z + self.w * right.w
    }
}

impl<T:alga::general::Real> FastDot<T> for Vec3<T>{
    #[inline]
    fn fast_dot(&self, right: &Vec3<T>) -> T{
        self.x * right.x + self.y * right.y + self.z * right.z
    }
}



pub trait FastMul<T>{
    type Output;
    fn fast_mul(&self, right: &T) -> Self::Output;
}

impl<T:alga::general::Real> FastMul<Mat4<T>> for Mat4<T>{
    type Output = Mat4<T>;
    #[inline]
    fn fast_mul(&self, right: &Mat4<T>) -> Mat4<T>{
        let row0 = [self.m11, self.m12, self.m13, self.m14];
        let row1 = [self.m21, self.m22, self.m23, self.m24];
        let row2 = [self.m31, self.m32, self.m33, self.m34];
        let row3 = [self.m41, self.m42, self.m43, self.m44];
        let col0 = [right.m11, right.m21, right.m31, right.m41];
        let col1 = [right.m12, right.m22, right.m32, right.m42];
        let col2 = [right.m13, right.m23, right.m33, right.m43];
        let col3 = [right.m14, right.m24, right.m34, right.m44];
        Mat4::new(
            row0.fast_dot(&col0), row0.fast_dot(&col1), row0.fast_dot(&col2), row0.fast_dot(&col3),
            row1.fast_dot(&col0), row1.fast_dot(&col1), row1.fast_dot(&col2), row1.fast_dot(&col3),
            row2.fast_dot(&col0), row2.fast_dot(&col1), row2.fast_dot(&col2), row2.fast_dot(&col3),
            row3.fast_dot(&col0), row3.fast_dot(&col1), row3.fast_dot(&col2), row3.fast_dot(&col3),
        )
    }
}

impl<T:alga::general::Real> FastMul<Vec4<T>> for Mat4<T>{
    type Output = Vec4<T>;
    #[inline]
    fn fast_mul(&self, right: &Vec4<T>) -> Vec4<T>{
        let row0 = [self.m11, self.m12, self.m13, self.m14];
        let row1 = [self.m21, self.m22, self.m23, self.m24];
        let row2 = [self.m31, self.m32, self.m33, self.m34];
        let row3 = [self.m41, self.m42, self.m43, self.m44];
        let right = right.as_ref();
        Vec4::new(
            row0.fast_dot(right), row1.fast_dot(right), row2.fast_dot(right), row3.fast_dot(right)
        )
    }
}

impl<T:alga::general::Real> FastMul<Mat3<T>> for Mat3<T>{
    type Output = Mat3<T>;
    #[inline]
    fn fast_mul(&self, right: &Mat3<T>) -> Mat3<T>{
        let row0 = [self.m11, self.m12, self.m13];
        let row1 = [self.m21, self.m22, self.m23];
        let row2 = [self.m31, self.m32, self.m33];
        let col0 = [right.m11, right.m21, right.m31];
        let col1 = [right.m12, right.m22, right.m32];
        let col2 = [right.m13, right.m23, right.m33];
        Mat3::new(
            row0.fast_dot(&col0), row0.fast_dot(&col1), row0.fast_dot(&col2),
            row1.fast_dot(&col0), row1.fast_dot(&col1), row1.fast_dot(&col2),
            row2.fast_dot(&col0), row2.fast_dot(&col1), row2.fast_dot(&col2),
        )
    }
}

impl<T:alga::general::Real> FastMul<Vec3<T>> for Mat3<T>{
    type Output = Vec3<T>;
    #[inline]
    fn fast_mul(&self, right: &Vec3<T>) -> Vec3<T>{
        let row0 = [self.m11, self.m12, self.m13];
        let row1 = [self.m21, self.m22, self.m23];
        let row2 = [self.m31, self.m32, self.m33];
        let right = right.as_ref();
        Vec3::new(
            row0.fast_dot(right), row1.fast_dot(right), row2.fast_dot(right)
        )
    }
}

pub trait FastInverse{
    fn fast_orthonormal_inverse(&self) -> Self;
    fn fast_affine_inverse(&self) -> Self;
}

impl<T:alga::general::Real> FastInverse for Mat4<T>{
    fn fast_orthonormal_inverse(&self) -> Mat4<T>{
        let _3x3 = Mat3::new(
            self.m11, self.m21, self.m31,
            self.m12, self.m22, self.m32,
            self.m13, self.m23, self.m33,
        );
        let pos = vec3(self.m14, self.m24, self.m34);
        let pos = -_3x3.fast_mul(&pos);
        Mat4::new(
            self.m11, self.m21, self.m31, pos.x,
            self.m12, self.m22, self.m32, pos.y,
            self.m13, self.m23, self.m33, pos.z,
            zero(),   zero(),   zero(),   one()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                ,
        )
    }

    fn fast_affine_inverse(&self) -> Mat4<T>{
        let _3x3 = Mat3::new(
            self.m11, self.m12, self.m13,
            self.m21, self.m22, self.m23,
            self.m31, self.m32, self.m33,
        ).try_inverse().unwrap();

        let pos = vec3(self.m14, self.m24, self.m34);
        let pos = -_3x3.fast_mul(&pos);
        Mat4::new(
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
                        alga::linear::FiniteDimVectorSpace +
                        PartialEq + Axpy<N>
                        where Self: Sized {
}

/// Trait of vector with components implementing the `BaseFloat` trait.
pub trait FloatVec<N: alga::general::Real>: NumVec<N> +
                        alga::linear::NormedSpace +
                        Neg<Output = Self> +
                        alga::linear::FiniteDimInnerSpace {
}


impl<N: BaseNum + alga::general::AbstractField + Neg<Output=N>> NumVec<N> for Vector1<N>{}
impl<N: BaseNum + alga::general::AbstractField + Neg<Output=N>> NumVec<N> for Vector2<N>{}
impl<N: BaseNum + alga::general::AbstractField + Neg<Output=N>> NumVec<N> for Vector3<N>{}
impl<N: BaseNum + alga::general::AbstractField + Neg<Output=N>> NumVec<N> for Vector4<N>{}
impl<N: BaseNum + alga::general::AbstractField + Neg<Output=N>> NumVec<N> for Vector5<N>{}
impl<N: BaseNum + alga::general::AbstractField + Neg<Output=N>> NumVec<N> for Vector6<N>{}

impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector1<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector2<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector3<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector4<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector5<N>{}
impl<N: BaseNum + alga::general::Real> FloatVec<N> for Vector6<N>{}
