# na

na is a wrapper around [nalgebra](http://nalgebra.org/) to make it easier and mostly less verbose to work with the most used types in vector math for graphics.

na shortens the most common types Vector*, Point*, Quaternion... to Vec*, Pnt*, Quat... and defaults the generic parameters to f32 so you can write:

```rust
struct Pos(Vec3);
```

It also inlcudes some other functionalities not yet present in nalgebra:

- Fast versions of multiplication of matrices (including matrices with vectors) that can be used by calling:

```rust
let m4: Mat4 = ...;
let v4: Vec4 = ...;
let v4_2 = m4.fast_mul(&v4);
```

- Fast versions of inversion for matrices

- GLSL style swizzles:

```rust
let xy = v3.xy()
let xz = v3.xz()
....
```

- Macros to easily create vectors:

```rust
let one = vec3!(1.); //sets all components to 1
let composed = vec3!(v2, z); // creates a Vector3 from a Vector2 and a float
...
```

- Easy to use traits for the most common types:

    - BaseNum (Integers and reals)
    - BaseInt
    - BaseFloat
    - NumVec (Vector of any number type)
    - FloatVec 
    - NumPnt
    - FloatPnt
