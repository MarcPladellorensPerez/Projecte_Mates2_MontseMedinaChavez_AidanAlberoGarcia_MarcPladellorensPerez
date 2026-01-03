#include "Matrix4x4.hpp"
#include <cmath>
#include <stdexcept>

#define TOL 1e-6

Matrix4x4 Matrix4x4::Identity()
{
    Matrix4x4 I;
    I.At(0, 0) = 1; I.At(1, 1) = 1; I.At(2, 2) = 1; I.At(3, 3) = 1;
    return I;
}

Matrix4x4 Matrix4x4::Multiply(const Matrix4x4& B) const
{
    Matrix4x4 C{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += At(i, k) * B.At(k, j);
            }
            C.At(i, j) = sum;
        }
    }
    return C;
}

Vec4 Matrix4x4::Multiply(const Vec4& v) const
{
    Vec4 res;
    res.x = At(0, 0) * v.x + At(0, 1) * v.y + At(0, 2) * v.z + At(0, 3) * v.w;
    res.y = At(1, 0) * v.x + At(1, 1) * v.y + At(1, 2) * v.z + At(1, 3) * v.w;
    res.z = At(2, 0) * v.x + At(2, 1) * v.y + At(2, 2) * v.z + At(2, 3) * v.w;
    res.w = At(3, 0) * v.x + At(3, 1) * v.y + At(3, 2) * v.z + At(3, 3) * v.w;
    return res;
}

// --------------------------------------------------------------------------
// LAB 3
// --------------------------------------------------------------------------

bool Matrix4x4::IsAffine() const
{
    if (std::abs(At(3, 0)) > TOL) return false;
    if (std::abs(At(3, 1)) > TOL) return false;
    if (std::abs(At(3, 2)) > TOL) return false;
    if (std::abs(At(3, 3) - 1.0) > TOL) return false;

    return true;
}

Vec3 Matrix4x4::TransformPoint(const Vec3& p) const
{
    Vec4 hp{ p.x, p.y, p.z, 1.0 };
    Vec4 tp = Multiply(hp);
    if (std::abs(tp.w) < TOL)
        throw std::runtime_error("Matrix4x4::TransformPoint: w component is zero");
    return { tp.x / tp.w, tp.y / tp.w, tp.z / tp.w };
}

Vec3 Matrix4x4::TransformVector(const Vec3& v) const
{
    Vec4 hv{ v.x, v.y, v.z, 0.0 };
    Vec4 tv = Multiply(hv);
    return { tv.x, tv.y, tv.z };
}

Matrix4x4 Matrix4x4::Translate(const Vec3& t)
{
    Matrix4x4 M = Identity();
    M.SetTranslation(t);
    return M;
}

Matrix4x4 Matrix4x4::Scale(const Vec3& s)
{
    Matrix4x4 M = Identity();
    M.At(0, 0) = s.x;
    M.At(1, 1) = s.y;
    M.At(2, 2) = s.z;
    return M;
}

Matrix4x4 Matrix4x4::Rotate(const Matrix3x3& R)
{
    Matrix4x4 M = Identity();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            M.At(i, j) = R.At(i, j);
    return M;
}

Matrix4x4 Matrix4x4::Rotate(const Quat& q)
{
    return Rotate(q.ToMatrix3x3());
}

Matrix4x4 Matrix4x4::FromTRS(const Vec3& t, const Matrix3x3& R, const Vec3& s)
{
    // M = T * R * S
    Matrix4x4 M{};

    // Bloc 3x3
    for (int i = 0; i < 3; ++i) {
        M.At(i, 0) = R.At(i, 0) * s.x;
        M.At(i, 1) = R.At(i, 1) * s.y;
        M.At(i, 2) = R.At(i, 2) * s.z;
    }

    // Columna 3
    M.At(0, 3) = t.x;
    M.At(1, 3) = t.y;
    M.At(2, 3) = t.z;

    // Element w
    M.At(3, 3) = 1.0;

    return M;
}

Matrix4x4 Matrix4x4::FromTRS(const Vec3& t, const Quat& q, const Vec3& s)
{
    return FromTRS(t, q.ToMatrix3x3(), s);
}

Matrix4x4 Matrix4x4::InverseTR() const
{
    // Assumeix Escala = 1.

    if (!IsAffine())
        throw std::runtime_error("InverseTR: Matrix is not affine (bottom row is not 0001)");

    Matrix3x3 R = GetRotationScale(); // S=1 => RS = R
    Matrix3x3 Rt = R.Transposed();
    Vec3 t = GetTranslation();

    Vec3 invT = Rt.Multiply(t);
    invT = { -invT.x, -invT.y, -invT.z };

    Matrix4x4 MInv = Rotate(Rt);
    MInv.SetTranslation(invT);
    return MInv;
}

Matrix4x4 Matrix4x4::InverseTRS() const
{
    // M^-1 = S^-1 * R^T * T^-1

    if (!IsAffine())
        throw std::runtime_error("InverseTRS: Matrix is not affine (bottom row is not 0001)");

	Vec3 s = GetScale();

    if (s.x < TOL || s.y < TOL || s.z < TOL)
        throw std::runtime_error("Matrix4x4::InverseTRS: Scale too close to zero");

    double isxSq = 1.0 / (s.x * s.x);
    double isySq = 1.0 / (s.y * s.y);
    double iszSq = 1.0 / (s.z * s.z);

    // Calculem A_inv = S^-1 * R^T
    // Equival a dividir la transposada pel quadrat de l'escala
    Matrix3x3 A_inv;
    // Fila 0
    A_inv.At(0, 0) = At(0, 0) * isxSq; A_inv.At(0, 1) = At(1, 0) * isxSq; A_inv.At(0, 2) = At(2, 0) * isxSq;
    // Fila 1
    A_inv.At(1, 0) = At(0, 1) * isySq; A_inv.At(1, 1) = At(1, 1) * isySq; A_inv.At(1, 2) = At(2, 1) * isySq;
    // Fila 2
    A_inv.At(2, 0) = At(0, 2) * iszSq; A_inv.At(2, 1) = At(1, 2) * iszSq; A_inv.At(2, 2) = At(2, 2) * iszSq;

    // Nova Translate: t' = - (A_inv * t)
    Vec3 t = GetTranslation();
    Vec3 invT = A_inv.Multiply(t);
    invT = { -invT.x, -invT.y, -invT.z };

    Matrix4x4 MInv = Rotate(A_inv);
    MInv.SetTranslation(invT);
    return MInv;
}

// --------------------------------------------------------------------------
// Getters
// --------------------------------------------------------------------------

Vec3 Matrix4x4::GetTranslation() const
{
    if (!IsAffine())
        throw std::runtime_error("Matrix is not affine (bottom row is not 0001)");

    return { At(0, 3), At(1, 3), At(2, 3) };
}

Matrix3x3 Matrix4x4::GetRotationScale() const
{
    if (!IsAffine())
        throw std::runtime_error("Matrix is not affine (bottom row is not 0001)");

    Matrix3x3 rs;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rs.At(i, j) = At(i, j);
    return rs;
}

Vec3 Matrix4x4::GetScale() const
{
    if (!IsAffine())
        throw std::runtime_error("Matrix is not affine (bottom row is not 0001)");

    double sx = Vec3{ At(0,0), At(1,0), At(2,0) }.Norm();
    double sy = Vec3{ At(0,1), At(1,1), At(2,1) }.Norm();
    double sz = Vec3{ At(0,2), At(1,2), At(2,2) }.Norm();

    if (sx < TOL || sy < TOL || sz < TOL)
        return { sx, sy, sz };

    Matrix3x3 R;
    // Normalitzem columnes dividint per escala
    R.At(0, 0) = At(0, 0) / sx;  R.At(0, 1) = At(0, 1) / sy;  R.At(0, 2) = At(0, 2) / sz;
    R.At(1, 0) = At(1, 0) / sx;  R.At(1, 1) = At(1, 1) / sy;  R.At(1, 2) = At(1, 2) / sz;
    R.At(2, 0) = At(2, 0) / sx;  R.At(2, 1) = At(2, 1) / sy;  R.At(2, 2) = At(2, 2) / sz;

	double detR = R.Det();
    // Assegurem que la matriu de rotacio resultant tingui determinant positiu
    if (std::abs(detR + 1.0) < TOL)
    {
		sx = -sx;
    }

    return { sx, sy, sz };
}

Matrix3x3 Matrix4x4::GetRotation() const
{
    if (!IsAffine())
        throw std::runtime_error("Matrix is not affine (bottom row is not 0001)");

    Vec3 s = GetScale();
    if (std::abs(s.x) < TOL || std::abs(s.y) < TOL || std::abs(s.z) < TOL) return Matrix3x3::Identity();

    Matrix3x3 R;
    // Normalitzem columnes dividint per escala
    R.At(0, 0) = At(0, 0) / s.x;  R.At(0, 1) = At(0, 1) / s.y;  R.At(0, 2) = At(0, 2) / s.z;
    R.At(1, 0) = At(1, 0) / s.x;  R.At(1, 1) = At(1, 1) / s.y;  R.At(1, 2) = At(1, 2) / s.z;
    R.At(2, 0) = At(2, 0) / s.x;  R.At(2, 1) = At(2, 1) / s.y;  R.At(2, 2) = At(2, 2) / s.z;
    
    return R;
}

Quat Matrix4x4::GetRotationQuat() const
{
    if (!IsAffine())
        throw std::runtime_error("Matrix is not affine (bottom row is not 0001)");

    return Quat::FromMatrix3x3(GetRotation());
}

// --------------------------------------------------------------------------
// Setters
// --------------------------------------------------------------------------

void Matrix4x4::SetTranslation(const Vec3& t)
{
    At(0, 3) = t.x;
    At(1, 3) = t.y;
    At(2, 3) = t.z;
}

void Matrix4x4::SetScale(const Vec3& s)
{
    Matrix3x3 R = GetRotation();

    for (int i = 0; i < 3; ++i) {
        At(i, 0) = R.At(i, 0) * s.x;
        At(i, 1) = R.At(i, 1) * s.y;
        At(i, 2) = R.At(i, 2) * s.z;
    }
}

void Matrix4x4::SetRotation(const Matrix3x3& R)
{
    Vec3 s = GetScale();

    for (int i = 0; i < 3; ++i) {
        At(i, 0) = R.At(i, 0) * s.x;
        At(i, 1) = R.At(i, 1) * s.y;
        At(i, 2) = R.At(i, 2) * s.z;
    }
}

void Matrix4x4::SetRotation(const Quat& q)
{
    SetRotation(q.ToMatrix3x3());
}

void Matrix4x4::SetRotationScale(const Matrix3x3& RS)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            At(i, j) = RS.At(i, j);
}