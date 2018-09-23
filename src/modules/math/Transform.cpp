/**
 * Copyright (c) 2006-2018 LOVE Development Team
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 **/

#include "Transform.h"

namespace love
{
namespace math
{

love::Type Transform::type("Transform", &Object::type);

Transform::Transform()
	: matrix()
	, inverseDirty(true)
	, frustumDirty(true)
	, inverseMatrix()
{
}

Transform::Transform(const Matrix4 &m)
	: matrix(m)
	, inverseDirty(true)
	, frustumDirty(true)
	, inverseMatrix()
{
}

Transform::Transform(float x, float y, float a, float sx, float sy, float ox, float oy, float kx, float ky)
	: matrix(x, y, a, sx, sy, ox, oy, kx, ky)
	, inverseDirty(true)
	, frustumDirty(true)
	, inverseMatrix()
{
}

Transform::~Transform()
{
}

Transform *Transform::clone()
{
	return new Transform(*this);
}

Transform *Transform::inverse()
{
	return new Transform(getInverseMatrix());
}

Transform *Transform::inverseTranspose()
{
	return new Transform(getInverseMatrix().transpose());
}

void Transform::apply(Transform *other)
{
	matrix *= other->getMatrix();
	markDirty();
}

void Transform::translate(float x, float y)
{
	matrix.translate(x, y);
	markDirty();
}

void Transform::translate(float x, float y, float z)
{
	matrix.translate(x, y, z);
	markDirty();
}

void Transform::rotate(float angle)
{
	matrix.rotate(angle);
	markDirty();
}

void Transform::rotate(float x, float y, float z, float angle)
{
	matrix.rotate(x, y, z, angle);
	markDirty();
}

void Transform::scale(float x, float y)
{
	matrix.scale(x, y);
	markDirty();
}

void Transform::scale(float x, float y, float z)
{
	matrix.scale(x, y, z);
	markDirty();
}

void Transform::shear(float x, float y)
{
	matrix.shear(x, y);
	markDirty();
}

void Transform::lookAt(float eyeX, float eyeY, float eyeZ, float targetX, float targetY, float targetZ, float upX, float upY, float upZ)
{
	matrix *= Matrix4::lookAt(eyeX, eyeY, eyeZ, targetX, targetY, targetZ, upX, upY, upZ);
	markDirty();
}

void Transform::perspective(float fieldOfView, float aspectRatio, float near, float far)
{
	matrix *= Matrix4::perspective(fieldOfView, aspectRatio, near, far);
	markDirty();
}

void Transform::ortho(float l, float r, float t, float b, float near, float far)
{
	matrix *= Matrix4::ortho(l, r, t, b, near, far);
	markDirty();
}

void Transform::reset()
{
	matrix.setIdentity();
	markDirty();
}

void Transform::setTransformation(float x, float y, float a, float sx, float sy, float ox, float oy, float kx, float ky)
{
	matrix.setTransformation(x, y, a, sx, sy, ox, oy, kx, ky);
	markDirty();
}

void Transform::fromQuaternion(float x, float y, float z, float w)
{
	matrix.fromQuaternion(x, y, z, w);
	markDirty();
}

void Transform::applyQuaternion(float x, float y, float z, float w)
{
	Matrix4 m;
	m.fromQuaternion(x, y, z, w);
	matrix *= m;
	markDirty();
}

love::Vector2 Transform::transformPoint(love::Vector2 p) const
{
	love::Vector2 result;
	matrix.transformXY(&result, &p, 1);
	return result;
}

love::Vector3 Transform::transformPoint(love::Vector3 p) const
{
	love::Vector3 result;
	matrix.transformXYZ(&result, &p, 1);
	return result;
}

love::Vector2 Transform::inverseTransformPoint(love::Vector2 p)
{
	love::Vector2 result;
	getInverseMatrix().transformXY(&result, &p, 1);
	return result;
}

love::Vector3 Transform::inverseTransformPoint(love::Vector3 p)
{
	love::Vector3 result;
	getInverseMatrix().transformXYZ(&result, &p, 1);
	return result;
}

const Matrix4 &Transform::getMatrix() const
{
	return matrix;
}

void Transform::setMatrix(const Matrix4 &m)
{
	matrix = m;
	markDirty();
}

bool Transform::getConstant(const char *in, MatrixLayout &out)
{
	return matrixLayouts.find(in, out);
}

bool Transform::getConstant(MatrixLayout in, const char *&out)
{
	return matrixLayouts.find(in, out);
}

std::vector<std::string> Transform::getConstants(MatrixLayout)
{
	return matrixLayouts.getNames();
}

static Vector3 getPositiveVertex(
	const Vector3 &min,
	const Vector3 &max,
	const Vector3 &normal)
{
	Vector3 result = min;
	if (normal.x >= 0)
	{
		result.x = max.x;
	}
	if (normal.y >= 0)
	{
		result.y = max.y;
	}
	if (normal.z >= 0)
	{
		result.z = max.z;
	}
	return result;
}

static Vector3 getNegativeVertex(
	const Vector3& min,
	const Vector3& max,
	const Vector3& normal)
{
	Vector3 result = max;
	if (normal.x >= 0)
	{
		result.x = min.x;
	}
	if (normal.y >= 0)
	{
		result.y = min.y;
	}
	if (normal.z >= 0)
	{
		result.z = min.z;
	}
	return result;
}

bool Transform::insideFrustum(const Vector3 &min, const Vector3 &max)
{
	Plane* frustum = getFrustum();

	for (int i = 0; i < NUM_PLANES; ++i)
	{
		Vector3 positiveVertex = getPositiveVertex(min, max, frustum[i].normal);
		float dot = Vector3::dot(positiveVertex, frustum[i].normal) + frustum[i].d;
		if (dot < 0)
		{
			return false;
		}
	}

	return true;
}

Transform::Plane *Transform::getFrustum()
{
	//  0:M11  4:M12  8:M13 12:M14
	//  1:M21  5:M22  9:M23 13:M24
	//  2:M31  6:M32 10:M33 14:M34
	//  3:M41  7:M42 11:M43 15:M44
	if (frustumDirty)
	{
		const float* m = matrix.getElements();
#define M(i, j) m[(j - 1) * 4 + (i - 1)]
		// left
		planes[0].normal.x = M(1, 1) + M(4, 1);
		planes[0].normal.y = M(1, 2) + M(4, 2);
		planes[0].normal.z = M(1, 3) + M(4, 3);
		planes[0].d        = M(1, 4) + M(4, 4);
		float leftLengthInverse = 1.0f / planes[0].normal.getLength();
		planes[0].normal  *= leftLengthInverse;
		planes[0].d       *= leftLengthInverse;

		// right
		planes[1].normal.x = -M(1, 1) + M(4, 1);
		planes[1].normal.y = -M(1, 2) + M(4, 2);
		planes[1].normal.z = -M(1, 3) + M(4, 3);
		planes[1].d        = -M(1, 4) + M(4, 4);
		float rightLengthInverse = 1.0f / planes[1].normal.getLength();
		planes[1].normal  *= rightLengthInverse;
		planes[1].d       *= rightLengthInverse;

		// top
		planes[2].normal.x = -M(2, 1) + M(4, 1);
		planes[2].normal.y = -M(2, 2) + M(4, 2);
		planes[2].normal.z = -M(2, 3) + M(4, 3);
		planes[2].d        = -M(2, 4) + M(4, 4);
		float topLengthInverse = 1.0f / planes[2].normal.getLength();
		planes[2].normal  *= topLengthInverse;
		planes[2].d       *= topLengthInverse;

		// bottom
		planes[3].normal.x = M(2, 1) + M(4, 1);
		planes[3].normal.y = M(2, 2) + M(4, 2);
		planes[3].normal.z = M(2, 3) + M(4, 3);
		planes[3].d        = M(2, 4) + M(4, 4);
		float bottomLengthInverse = 1.0f / planes[3].normal.getLength();
		planes[3].normal  *= bottomLengthInverse;
		planes[3].d       *= bottomLengthInverse;

		// near
		planes[4].normal.x = M(3, 1) + M(4, 1);
		planes[4].normal.y = M(3, 2) + M(4, 2);
		planes[4].normal.z = M(3, 3) + M(4, 3);
		planes[4].d        = M(3, 4) + M(4, 4);
		float nearLengthInverse = 1.0f / planes[4].normal.getLength();
		planes[4].normal  *= nearLengthInverse;
		planes[4].d       *= nearLengthInverse;

		// far
		planes[5].normal.x = -M(3, 1) + M(4, 1);
		planes[5].normal.y = -M(3, 2) + M(4, 2);
		planes[5].normal.z = -M(3, 3) + M(4, 3);
		planes[5].d        = -M(3, 4) + M(4, 4);
		float farLengthInverse = 1.0f / planes[5].normal.getLength();
		planes[5].normal  *= farLengthInverse;
		planes[5].d       *= farLengthInverse;
#undef M
		frustumDirty = false;
	}

	return planes;
}

StringMap<Transform::MatrixLayout, Transform::MATRIX_MAX_ENUM>::Entry Transform::matrixLayoutEntries[] =
{
	{ "row",    MATRIX_ROW_MAJOR    },
	{ "column", MATRIX_COLUMN_MAJOR },
};

StringMap<Transform::MatrixLayout, Transform::MATRIX_MAX_ENUM> Transform::matrixLayouts(Transform::matrixLayoutEntries, sizeof(Transform::matrixLayoutEntries));

} // math
} // love
