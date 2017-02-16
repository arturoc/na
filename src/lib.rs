extern crate nalgebra as na;
extern crate alga;

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

use std::fmt::Debug;

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
