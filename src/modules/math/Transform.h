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

#pragma once

// LOVE
#include "common/Object.h"
#include "common/Matrix.h"
#include "common/Vector.h"
#include "common/StringMap.h"

namespace love
{
namespace math
{

class Transform : public Object
{
public:

	enum MatrixLayout
	{
		MATRIX_ROW_MAJOR,
		MATRIX_COLUMN_MAJOR,
		MATRIX_MAX_ENUM
	};

	static love::Type type;

	Transform();
	Transform(const Matrix4 &m);
	Transform(float x, float y, float a, float sx, float sy, float ox, float oy, float kx, float ky);

	virtual ~Transform();

	Transform *clone();
	Transform *inverse();
	Transform *inverseTranspose();

	void apply(Transform *other);

	void translate(float x, float y);
	void translate(float x, float y, float z);
	void rotate(float angle);
	void rotate(float x, float y, float z, float angle);
	void scale(float x, float y);
	void scale(float x, float y, float z);
	void shear(float x, float y);
	void shear(float x, float y, float z);

	void lookAt(float eyeX, float eyeY, float eyeZ, float targetX, float targetY, float targetZ, float upX, float upY, float upZ);
	void perspective(float fieldOfView, float aspectRatio, float near, float far);
	void ortho(float l, float r, float t, float b, float near, float far);

	void reset();
	void setTransformation(float x, float y, float a, float sx, float sy, float ox, float oy, float kx, float ky);

	void fromQuaternion(float x, float y, float z, float w);
	void applyQuaternion(float x, float y, float z, float w);

	love::Vector2 transformPoint(love::Vector2 p) const;
	love::Vector3 transformPoint(love::Vector3 p) const;
	love::Vector2 inverseTransformPoint(love::Vector2 p);
	love::Vector3 inverseTransformPoint(love::Vector3 p);

	const Matrix4 &getMatrix() const;
	void setMatrix(const Matrix4 &m);

	static bool getConstant(const char *in, MatrixLayout &out);
	static bool getConstant(MatrixLayout in, const char *&out);
	static std::vector<std::string> getConstants(MatrixLayout);

	bool insideFrustum(const Vector3& min, const Vector3& max);

private:

	inline void markDirty()
	{
		inverseDirty = true;
		frustumDirty = true;
	}

	inline const Matrix4 &getInverseMatrix()
	{
		if (inverseDirty)
		{
			inverseDirty = false;
			inverseMatrix = matrix.inverse();
		}

		return inverseMatrix;
	}

	Matrix4 matrix;
	bool inverseDirty;
	Matrix4 inverseMatrix;

	static const int NUM_PLANES = 6;
	struct Plane { Vector3 normal; float d; };
	Plane planes[NUM_PLANES];
	bool frustumDirty;
	Plane *getFrustum();

	static StringMap<MatrixLayout, MATRIX_MAX_ENUM>::Entry matrixLayoutEntries[];
	static StringMap<MatrixLayout, MATRIX_MAX_ENUM> matrixLayouts;

}; // Transform


} // math
} // love
