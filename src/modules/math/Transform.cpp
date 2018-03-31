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
	, inverseMatrix()
{
}

Transform::Transform(const Matrix4 &m)
	: matrix(m)
	, inverseDirty(true)
	, inverseMatrix()
{
}

Transform::Transform(float x, float y, float a, float sx, float sy, float ox, float oy, float kx, float ky)
	: matrix(x, y, a, sx, sy, ox, oy, kx, ky)
	, inverseDirty(true)
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

void Transform::apply(Transform *other)
{
	matrix *= other->getMatrix();
	inverseDirty = true;
}

void Transform::translate(float x, float y)
{
	matrix.translate(x, y);
	inverseDirty = true;
}

void Transform::translate(float x, float y, float z)
{
	matrix.translate(x, y, z);
	inverseDirty = true;
}

void Transform::rotate(float angle)
{
	matrix.rotate(angle);
	inverseDirty = true;
}

void Transform::rotate(float x, float y, float z, float angle)
{
	matrix.rotate(x, y, z, angle);
	inverseDirty = true;
}

void Transform::scale(float x, float y)
{
	matrix.scale(x, y);
	inverseDirty = true;
}

void Transform::scale(float x, float y, float z)
{
	matrix.scale(x, y, z);
	inverseDirty = true;
}

void Transform::shear(float x, float y)
{
	matrix.shear(x, y);
	inverseDirty = true;
}

void Transform::reset()
{
	matrix.setIdentity();
	inverseDirty = true;
}

void Transform::setTransformation(float x, float y, float a, float sx, float sy, float ox, float oy, float kx, float ky)
{
	matrix.setTransformation(x, y, a, sx, sy, ox, oy, kx, ky);
	inverseDirty = true;
}

void Transform::fromQuaternion(float x, float y, float z, float w)
{
	matrix.fromQuaternion(x, y, z, w);
	inverseDirty = true;
}

void Transform::applyQuaternion(float x, float y, float z, float w)
{
	Matrix4 m;
	m.fromQuaternion(x, y, z, w);
	matrix *= m;
	inverseDirty = true;
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
	inverseDirty = true;
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

StringMap<Transform::MatrixLayout, Transform::MATRIX_MAX_ENUM>::Entry Transform::matrixLayoutEntries[] =
{
	{ "row",    MATRIX_ROW_MAJOR    },
	{ "column", MATRIX_COLUMN_MAJOR },
};

StringMap<Transform::MatrixLayout, Transform::MATRIX_MAX_ENUM> Transform::matrixLayouts(Transform::matrixLayoutEntries, sizeof(Transform::matrixLayoutEntries));

} // math
} // love
