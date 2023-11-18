//
//

#include <vector>
#include <fstream>
#include <strstream>
#include <algorithm>

#include "olcConsoleGameEngine.h"

//

struct Vec2D {
	float u = 0;
	float v = 0;
};

struct Vec3D {
	float x = 0;
	float y = 0;
	float z = 0;
	float w = 1; // Need a 4th term to perform sensible matrix-vector multiplication
};

struct Triangle {
	Vec3D t[3]; // Triangle
	Vec2D tx[3]; // Texture
	wchar_t symbol;
	short colour;
};

struct Mesh {
	std::vector<Triangle> tris;
	bool LoadFromObjectFile(std::string sFilename) {
		std::ifstream f(sFilename);
		if (!f.is_open()) {
			return false;
		}

		// Local cache of vertices
		std::vector<Vec3D> verts;
		while (!f.eof()) {
			char line[128];
			f.getline(line, 128);

			std::strstream s;
			s << line;

			char junk;
			if (line[0] == 'v') {
				Vec3D vert;
				s >> junk >> vert.x >> vert.y >> vert.z;
				verts.push_back(vert);
			}
			if (line[0] == 'f') {
				int f[3];
				s >> junk >> f[0] >> f[1] >> f[2];
				tris.push_back({ verts[f[0] - 1], verts[f[1] - 1], verts[f[2] - 1] });
			}
		}

		return true;
	}
};

struct Mat4x4 {
	float m[4][4] = { 0 };
};

class GraphicsEngine3D :public olcConsoleGameEngine {
private:
	Mesh meshObject;
	Mat4x4 matProjection;
	float fTheta;
	Vec3D vCamera;
	Vec3D vLookDir;
	float fYaw;

	CHAR_INFO GetColour(float luminance) {
		short bgColour, fgColour;
		wchar_t symbol;
		int pixColourShading = (int)(13.0f * luminance);
		switch (pixColourShading) {
		case 0: bgColour = BG_BLACK; fgColour = FG_BLACK; symbol = PIXEL_SOLID; break;

		case 1: bgColour = BG_BLACK; fgColour = FG_DARK_GREY; symbol = PIXEL_QUARTER; break;
		case 2: bgColour = BG_BLACK; fgColour = FG_DARK_GREY; symbol = PIXEL_HALF; break;
		case 3: bgColour = BG_BLACK; fgColour = FG_DARK_GREY; symbol = PIXEL_THREEQUARTERS; break;
		case 4: bgColour = BG_BLACK; fgColour = FG_DARK_GREY; symbol = PIXEL_SOLID; break;

		case 5: bgColour = BG_DARK_GREY; fgColour = FG_GREY; symbol = PIXEL_QUARTER; break;
		case 6: bgColour = BG_DARK_GREY; fgColour = FG_GREY; symbol = PIXEL_HALF;
		case 7: bgColour = BG_DARK_GREY; fgColour = FG_GREY; symbol = PIXEL_THREEQUARTERS; break;
		case 8: bgColour = BG_DARK_GREY; fgColour = FG_GREY; symbol = PIXEL_SOLID; break;

		case 9:  bgColour = BG_GREY; fgColour = FG_WHITE; symbol = PIXEL_QUARTER; break;
		case 10: bgColour = BG_GREY; fgColour = FG_WHITE; symbol = PIXEL_HALF; break;
		case 11: bgColour = BG_GREY; fgColour = FG_WHITE; symbol = PIXEL_THREEQUARTERS; break;
		case 12: bgColour = BG_GREY; fgColour = FG_WHITE; symbol = PIXEL_SOLID; break;

		default: bgColour = BG_BLACK; fgColour = FG_BLACK; symbol = PIXEL_SOLID;
		}

		CHAR_INFO c;
		c.Attributes = bgColour | fgColour;
		c.Char.UnicodeChar = symbol;

		return c;
	}

	Vec3D VecsAdd(Vec3D& v1, Vec3D& v2) {
		return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
	}
	Vec3D VecsSubtract(Vec3D& v1, Vec3D& v2) {
		return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
	}
	Vec3D VecsMultiply(Vec3D& v, float k) {
		return{ v.x * k, v.y * k, v.z * k };
	}
	Vec3D VecsDivide(Vec3D& v, float k) {
		return{ v.x / k, v.y / k, v.z / k };
	}

	float VecsDotProduct(Vec3D& v1, Vec3D& v2) {
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}
	float VecLength(Vec3D& v) {
		return sqrtf(VecsDotProduct(v, v));
	}

	Vec3D VecNormalise(Vec3D& v) {
		float l = VecLength(v);
		return { v.x / l, v.y / l, v.z / l };
	}
	Vec3D VecsCrossProduct(Vec3D& v1, Vec3D& v2) {
		Vec3D v;
		v.x = v1.y * v2.z - v1.z * v2.y;
		v.y = v1.z * v2.x - v1.x * v2.z;
		v.z = v1.x * v2.y - v1.y * v2.x;
		return v;
	}

	Vec3D VecIntersectPlane(Vec3D& vPlanePoint, Vec3D& vPlaneNormal, Vec3D& vLineStart, Vec3D& vLineEnd, float& t) {
		vPlaneNormal = VecNormalise(vPlaneNormal);

		// Vector notation of line-plane intersection
		Vec3D v = VecsSubtract(vPlanePoint, vLineStart);
		float n = VecsDotProduct(v, vPlaneNormal);
		Vec3D vLine = VecsSubtract(vLineEnd, vLineStart);
		float d = VecsDotProduct(vLine, vPlaneNormal);
		t = n / d; // Normalised distance along the line between two points where the intersection happened
		Vec3D vLineToIntersect = VecsMultiply(vLine, t);

		return VecsAdd(vLineStart, vLineToIntersect);
	}

	int TriangleClipAgainstPlane(Vec3D vPlanePoint, Vec3D vPlaneNormal, Triangle& triIn, Triangle& triOut1, Triangle& triOut2) {
		vPlaneNormal = VecNormalise(vPlaneNormal);

		// Retunrn signed shortest distance from point to plane (plane's normal vector must be normalised)
		auto dist = [&](Vec3D& p) {
			Vec3D n = VecNormalise(p);
			// If distance > 0: point lies inside; distance < 0: point lies outside
			return (vPlaneNormal.x * p.x + vPlaneNormal.y * p.y + vPlaneNormal.z * p.z - VecsDotProduct(vPlaneNormal, vPlanePoint));
			};

		Vec3D* inPoints[3]; int nInPointCount = 0;
		Vec3D* outPoints[3]; int nOutPointCount = 0;
		Vec2D* inTextures[3]; int nInTextureCount = 0;
		Vec2D* outTextures[3]; int nOutTextureCount = 0;

		// Get signed distance of each point in triangle to plane
		float d0 = dist(triIn.t[0]);
		float d1 = dist(triIn.t[1]);
		float d2 = dist(triIn.t[2]);
		if (d0 >= 0) { 
			inPoints[nInPointCount++] = &triIn.t[0]; 
			inTextures[nInTextureCount++] = &triIn.tx[0];
		}
		else { 
			outPoints[nOutPointCount++] = &triIn.t[0]; 
			outTextures[nOutTextureCount++] = &triIn.tx[0];
		}
		if (d1 >= 0) { 
			inPoints[nInPointCount++] = &triIn.t[1]; 
			inTextures[nInTextureCount++] = &triIn.tx[1];
		}
		else { 
			outPoints[nOutPointCount++] = &triIn.t[1]; 
			outTextures[nOutTextureCount++] = &triIn.tx[1];
		}
		if (d2 >= 0) { 
			inPoints[nInPointCount++] = &triIn.t[2]; 
			inTextures[nInTextureCount++] = &triIn.tx[2];
		}
		else { 
			outPoints[nOutPointCount++] = &triIn.t[2]; 
			outTextures[nOutTextureCount++] = &triIn.tx[2];
		}

		if (nInPointCount == 0) {
			// All points lie outside of plane, so clip whole triangle
			return 0;
		}
		if (nInPointCount == 3) {
			// All points lie inside of plane, so allow the triangle to pass through
			triOut1 = triIn;
			return 1;
		}
		if (nInPointCount == 1 && nOutPointCount == 2) {
			triOut1.colour = triIn.colour;
			triOut1.symbol = triIn.symbol;

			// 2 points lie outside of plane, so the triangle is clipped to a smaller triangle
			// Keep 1 inside point
			triOut1.t[0] = *inPoints[0];
			triOut1.tx[0] = *inTextures[0];
			// Construct 2 new points intersect with the plane
			float t;
			triOut1.t[1] = VecIntersectPlane(vPlanePoint, vPlaneNormal, *inPoints[0], *outPoints[0], t);
			// Calculate texture coordinates: distance + offset by starting point
			triOut1.tx[1].u = t * (outTextures[0]->u - inTextures[0]->u) + inTextures[0]->u;
			triOut1.tx[1].v = t * (outTextures[0]->v - inTextures[0]->v) + inTextures[0]->v;
			triOut1.t[2] = VecIntersectPlane(vPlanePoint, vPlaneNormal, *inPoints[0], *outPoints[1], t);
			triOut1.tx[2].u = t * (outTextures[1]->u - inTextures[0]->u) + inTextures[0]->u;
			triOut1.tx[2].v = t * (outTextures[1]->v - inTextures[0]->v) + inTextures[0]->v;

			return 1;
		}
		if (nInPointCount == 2 && nOutPointCount == 1) {
			triOut1.colour = triIn.colour;
			triOut1.symbol = triIn.symbol;
			triOut2.colour = triIn.colour;
			triOut2.symbol = triIn.symbol;

			// 2 points lie inside of the plane -> the triangle is clipped to a "quad". We represent it with 2 new triangles
			// Triangle 1 consists of 2 inside points and a new point determined by the intersection of 1 original triangle with the plane
			triOut1.t[0] = *inPoints[0];
			triOut1.t[1] = *inPoints[1];
			triOut1.tx[0] = *inTextures[0];
			triOut1.tx[1] = *inTextures[1];

			float t;
			triOut1.t[2] = VecIntersectPlane(vPlanePoint, vPlaneNormal, *inPoints[0], *outPoints[0], t);
			triOut1.tx[2].u = t * (outTextures[0]->u - inTextures[0]->u) + inTextures[0]->u;
			triOut1.tx[2].v = t * (outTextures[0]->v - inTextures[0]->v) + inTextures[0]->v;

			// Triangle 2 consists of 1 inside point and two intersected points 
			triOut2.t[0] = *inPoints[1];
			triOut2.tx[0] = *inTextures[1];
			triOut2.t[1] = triOut1.t[2];
			triOut2.tx[1] = triOut1.tx[2];
			triOut2.t[2] = VecIntersectPlane(vPlanePoint, vPlaneNormal, *inPoints[1], *outPoints[0], t);
			triOut2.tx[2].u = t * (outTextures[0]->u - inTextures[1]->u) + inTextures[1]->u;
			triOut2.tx[2].v = t * (outTextures[0]->v - inTextures[1]->v) + inTextures[1]->v;

			return 2;
		}
	}

	Mat4x4 MatMakeIdentity() {
		Mat4x4 matrix;
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		return matrix;
	}
	Mat4x4 MatMakeRotationX(float fAngleRad) {
		Mat4x4 matrix;
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = cosf(fAngleRad);
		matrix.m[1][2] = sinf(fAngleRad);
		matrix.m[2][1] = -sinf(fAngleRad);
		matrix.m[2][2] = cosf(fAngleRad);
		matrix.m[3][3] = 1.0f;
		return matrix;
	}
	Mat4x4 MatMakeRotationY(float fAngleRad) {
		Mat4x4 matrix;
		matrix.m[0][0] = cosf(fAngleRad);
		matrix.m[0][2] = sinf(fAngleRad);
		matrix.m[2][0] = -sinf(fAngleRad);
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = cosf(fAngleRad);
		matrix.m[3][3] = 1.0f;
		return matrix;
	}
	Mat4x4 MatMakeRotationZ(float fAngleRad) {
		Mat4x4 matrix;
		matrix.m[0][0] = cosf(fAngleRad);
		matrix.m[0][1] = sinf(fAngleRad);
		matrix.m[1][0] = -sinf(fAngleRad);
		matrix.m[1][1] = cosf(fAngleRad);
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		return matrix;
	}
	Mat4x4 MatMakeProjection(float fFOVDeg, float fAspectRatio, float fNear, float fFar) {
		float fFOVRad = 1.0f / tanf(fFOVDeg * 0.5f / 180.0f * 3.1415926f);
		Mat4x4 matrix;
		matrix.m[0][0] = fAspectRatio * fFOVRad;
		matrix.m[1][1] = fFOVRad;
		matrix.m[2][2] = fFar / (fFar - fNear);
		matrix.m[3][2] = (-fFar * fNear) / (fFar - fNear);
		matrix.m[2][3] = -1.0f;
		matrix.m[3][3] = 0.0f;
		return matrix;
	}
	Mat4x4 MatMakeTranslation(float x, float y, float z) {
		Mat4x4 matrix;
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		matrix.m[3][0] = x;
		matrix.m[3][1] = y;
		matrix.m[3][2] = z;
		return matrix;
	}

	Vec3D MultiplyMatVec(Mat4x4& m, Vec3D& vi) {
		Vec3D vo;
		vo.x = vi.x * m.m[0][0] + vi.y * m.m[1][0] + vi.z * m.m[2][0] + vi.w * m.m[3][0];
		vo.y = vi.x * m.m[0][1] + vi.y * m.m[1][1] + vi.z * m.m[2][1] + vi.w * m.m[3][1];
		vo.z = vi.x * m.m[0][2] + vi.y * m.m[1][2] + vi.z * m.m[2][2] + vi.w * m.m[3][2];
		vo.w = vi.x * m.m[0][3] + vi.y * m.m[1][3] + vi.z * m.m[2][3] + vi.w * m.m[3][3];
		return vo;
	}
	Mat4x4 MultiplyMatMat(Mat4x4& m1, Mat4x4& m2) {
		Mat4x4 matrix;
		for (int c = 0; c < 4; c++) {
			for (int r = 0; r < 4; r++) {
				matrix.m[r][c] = m1.m[r][0] * m2.m[0][c] + m1.m[r][1] * m2.m[1][c] + m1.m[r][2] * m2.m[2][c] + m1.m[r][3] * m2.m[3][c];
			}
		}
		return matrix;
	}

	Mat4x4 MatPointAt(Vec3D& vPos, Vec3D& vTarget, Vec3D& vUp) {
		// Calculate new forward direction
		Vec3D vNewForward = VecsSubtract(vTarget, vPos);
		vNewForward = VecNormalise(vNewForward);

		// Create new up direction
		// Calculate original up vector projected on new forward vector
		Vec3D v = VecsMultiply(vNewForward, VecsDotProduct(vUp, vNewForward));
		Vec3D vNewUp = VecsSubtract(vUp, v);
		vNewUp = VecNormalise(vNewUp);

		// Create new right direction
		Vec3D vNewRight = VecsCrossProduct(vNewUp, vNewForward);
		//vNewRight = VecNormalise(vNewRight);

		// Construct dimensioning and translation matrix
		Mat4x4 matrix;
		matrix.m[0][0] = vNewRight.x;	  matrix.m[0][1] = vNewRight.y;	  matrix.m[0][2] = vNewRight.z;	  matrix.m[0][3] = 0.0f;
		matrix.m[1][0] = vNewUp.x;		  matrix.m[1][1] = vNewUp.y;		  matrix.m[1][2] = vNewUp.z;		  matrix.m[1][3] = 0.0f;
		matrix.m[2][0] = vNewForward.x;	matrix.m[2][1] = vNewForward.y;	matrix.m[2][2] = vNewForward.z;	matrix.m[2][3] = 0.0f;
		matrix.m[3][0] = vPos.x;			  matrix.m[3][1] = vPos.y;			  matrix.m[3][2] = vPos.z;			  matrix.m[3][3] = 1.0f;
		return matrix;
	}
	// We can get the "LookAt" matrix by inverting the "PointAt" matrix (only for rotation/translation matrices)
	Mat4x4 MatQuickInverse(Mat4x4& m) {
		Mat4x4 matrix;
		matrix.m[0][0] = m.m[0][0]; matrix.m[0][1] = m.m[1][0]; matrix.m[0][2] = m.m[2][0]; matrix.m[0][3] = 0.0f;
		matrix.m[1][0] = m.m[0][1]; matrix.m[1][1] = m.m[1][1]; matrix.m[1][2] = m.m[2][1]; matrix.m[1][3] = 0.0f;
		matrix.m[2][0] = m.m[0][2]; matrix.m[2][1] = m.m[1][2]; matrix.m[2][2] = m.m[2][2]; matrix.m[2][3] = 0.0f;
		matrix.m[3][0] = -(m.m[3][0] * matrix.m[0][0] + m.m[3][1] * matrix.m[1][0] + m.m[3][2] * matrix.m[2][0]);
		matrix.m[3][1] = -(m.m[3][0] * matrix.m[0][1] + m.m[3][1] * matrix.m[1][1] + m.m[3][2] * matrix.m[2][1]);
		matrix.m[3][2] = -(m.m[3][0] * matrix.m[0][2] + m.m[3][1] * matrix.m[1][2] + m.m[3][2] * matrix.m[2][2]);
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

public:
	GraphicsEngine3D() {
		m_sAppName = L"3D Graphics Engine";
	};

public:
	bool OnUserCreate() override {
		//meshObject.LoadFromObjectFile("mountains.obj");
		meshObject.tris = {
			// SOUTH
			{ 0.0f, 0.0f, 0.0f, 1.0f,    0.0f, 1.0f, 0.0f, 1.0f,    1.0f, 1.0f, 0.0f, 1.0f,    0.0f, 1.0f,    0.0f, 0.0f,    1.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f,    1.0f, 1.0f, 0.0f, 1.0f,    1.0f, 0.0f, 0.0f, 1.0f,    0.0f, 1.0f,    1.0f, 0.0f,    1.0f, 1.0f },
																																										     
			// EAST           																														     
			{ 1.0f, 0.0f, 0.0f, 1.0f,    1.0f, 1.0f, 0.0f, 1.0f,    1.0f, 1.0f, 1.0f, 1.0f,    0.0f, 1.0f,    0.0f, 0.0f,    1.0f, 0.0f },
			{ 1.0f, 0.0f, 0.0f, 1.0f,    1.0f, 1.0f, 1.0f, 1.0f,    1.0f, 0.0f, 1.0f, 1.0f,    0.0f, 1.0f,    1.0f, 0.0f,    1.0f, 1.0f },
																																										     
			// NORTH        																															     
			{ 1.0f, 0.0f, 1.0f, 1.0f,    1.0f, 1.0f, 1.0f, 1.0f,    0.0f, 1.0f, 1.0f, 1.0f,    0.0f, 1.0f,    0.0f, 0.0f,    1.0f, 0.0f },
			{ 1.0f, 0.0f, 1.0f, 1.0f,    0.0f, 1.0f, 1.0f, 1.0f,    0.0f, 0.0f, 1.0f, 1.0f,    0.0f, 1.0f,    1.0f, 0.0f,    1.0f, 1.0f },
																																										     
			// WEST           																														     
			{ 0.0f, 0.0f, 1.0f, 1.0f,    0.0f, 1.0f, 1.0f, 1.0f,    0.0f, 1.0f, 0.0f, 1.0f,    0.0f, 1.0f,    0.0f, 0.0f,    1.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f, 1.0f,    0.0f, 1.0f, 0.0f, 1.0f,    0.0f, 0.0f, 0.0f, 1.0f,    0.0f, 1.0f,    1.0f, 0.0f,    1.0f, 1.0f },
																																										     
			// TOP          																															     
			{ 0.0f, 1.0f, 0.0f, 1.0f,    0.0f, 1.0f, 1.0f, 1.0f,    1.0f, 1.0f, 1.0f, 1.0f,    0.0f, 1.0f,    0.0f, 0.0f,    1.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f, 1.0f,    1.0f, 1.0f, 1.0f, 1.0f,    1.0f, 1.0f, 0.0f, 1.0f,    0.0f, 1.0f,    1.0f, 0.0f,    1.0f, 1.0f },
																																										     
			// BOTTOM         																														     
			{ 1.0f, 0.0f, 1.0f, 1.0f,    0.0f, 0.0f, 1.0f, 1.0f,    0.0f, 0.0f, 0.0f, 1.0f,    0.0f, 1.0f,    0.0f, 0.0f,    1.0f, 0.0f },
			{ 1.0f, 0.0f, 1.0f, 1.0f,    0.0f, 0.0f, 0.0f, 1.0f,    1.0f, 0.0f, 0.0f, 1.0f,    0.0f, 1.0f,    1.0f, 0.0f,    1.0f, 1.0f },
		};

		// Projection matrix
		matProjection = MatMakeProjection(90.0f, (float)ScreenHeight() / (float)ScreenWidth(), 0.1f, 1000.0f);

		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override {
		// Control camera using keyboard
		if (GetKey(VK_UP).bHeld) {
			vCamera.y += 8.0f * fElapsedTime;
		}
		if (GetKey(VK_DOWN).bHeld) {
			vCamera.y -= 8.0f * fElapsedTime;
		}
		if (GetKey(VK_LEFT).bHeld) {
			vCamera.x -= 8.0f * fElapsedTime;
		}
		if (GetKey(VK_RIGHT).bHeld) {
			vCamera.x += 8.0f * fElapsedTime;
		}

		// Look forward and backward -> Scaling by the looking direction
		Vec3D vForward = VecsMultiply(vLookDir, 8.0f * fElapsedTime);
		if (GetKey(L'W').bHeld) {
			vCamera = VecsAdd(vCamera, vForward);
		}
		if (GetKey(L'S').bHeld) {
			vCamera = VecsSubtract(vCamera, vForward);
		}
		if (GetKey(L'A').bHeld) {
			fYaw -= 2.0f * fElapsedTime;
		}
		if (GetKey(L'D').bHeld) {
			fYaw += 2.0f * fElapsedTime;
		}

		Fill(0, 0, ScreenWidth(), ScreenHeight(), PIXEL_SOLID, FG_BLACK);

		Mat4x4 matRotationZ, matRotationX, matRotation, matTranslation, matTransformation;
		//fTheta += 1.0f * fElapsedTime;

		// Object transformation: rotation 1st, translation 2nd
		matRotationZ = MatMakeRotationZ(fTheta * 0.5f);
		matRotationX = MatMakeRotationX(fTheta);

		matTranslation = MatMakeTranslation(0.0f, 0.0f, 5.0f);

		matTransformation = MatMakeIdentity();
		matTransformation = MultiplyMatMat(matRotationZ, matRotationX);
		matTransformation = MultiplyMatMat(matTransformation, matTranslation);

		Vec3D vUp = { 0,1,0 };
		Vec3D vTarget = { 0,0,1 };
		Mat4x4 matCameratRotationY = MatMakeRotationY(fYaw);
		// New look direction after rotating camera
		vLookDir = MultiplyMatVec(matCameratRotationY, vTarget);
		vTarget = VecsAdd(vCamera, vLookDir);
		Mat4x4 matCamera = MatPointAt(vCamera, vTarget, vUp);
		// Make view matrix from camera
		Mat4x4 matView = MatQuickInverse(matCamera);

		std::vector<Triangle> vecTrianglesToRaster;

		// Draw triangles
		for (auto tri : meshObject.tris) {
			Triangle triProjected, triTransformed, triViewed;

			// Transformation (rotation + translation)
			triTransformed.t[0] = MultiplyMatVec(matTransformation, tri.t[0]);
			triTransformed.t[1] = MultiplyMatVec(matTransformation, tri.t[1]);
			triTransformed.t[2] = MultiplyMatVec(matTransformation, tri.t[2]);
			// Copy texture
			triTransformed.tx[0] = tri.tx[0];
			triTransformed.tx[1] = tri.tx[1];
			triTransformed.tx[2] = tri.tx[2];

			// Calculate cross products
			Vec3D normal, line1, line2;

			line1 = VecsSubtract(triTransformed.t[1], triTransformed.t[0]);
			line2 = VecsSubtract(triTransformed.t[2], triTransformed.t[0]);

			normal = VecsCrossProduct(line1, line2);
			normal = VecNormalise(normal);

			// Projection from 3D to 2D
			Vec3D vCameraRays = VecsSubtract(triTransformed.t[0], vCamera);

			if (VecsDotProduct(normal, vCameraRays) < 0.0f) {
				// Illumination
				Vec3D lightDirection = { 0.0f, 1.0f, -1.0f }; // single direction light
				lightDirection = VecNormalise(lightDirection);
				
				// How "aligned" are light direction and triangle surface normal?
				float dp = max(0.1f, VecsDotProduct(lightDirection, normal));
				// Extract colour and shading of grey combination (very console-specific!)
				CHAR_INFO colourShading = GetColour(dp);
				triTransformed.colour = colourShading.Attributes;
				triTransformed.symbol = colourShading.Char.UnicodeChar;

				// Convert world space to view space before projection
				triViewed.t[0] = MultiplyMatVec(matView, triTransformed.t[0]);
				triViewed.t[1] = MultiplyMatVec(matView, triTransformed.t[1]);
				triViewed.t[2] = MultiplyMatVec(matView, triTransformed.t[2]);
				triViewed.colour = triTransformed.colour;
				triViewed.symbol = triTransformed.symbol;
				// Copy texture
				triViewed.tx[0] = triTransformed.tx[0];
				triViewed.tx[1] = triTransformed.tx[1];
				triViewed.tx[2] = triTransformed.tx[2];

				// Clip viewed triangle against near plane -> this forms 2 additional triangles
				int nClippedTriangles = 0;
				Triangle triClipped[2];
				nClippedTriangles = TriangleClipAgainstPlane({ 0.0f, 0.0f, 0.1f }, { 0.0f, 0.0f, 1.0f }, triViewed, triClipped[0], triClipped[1]);

				for (int n = 0; n < nClippedTriangles; n++) {
					// Projection
					triProjected.t[0] = MultiplyMatVec(matProjection, triClipped[n].t[0]);
					triProjected.t[1] = MultiplyMatVec(matProjection, triClipped[n].t[1]);
					triProjected.t[2] = MultiplyMatVec(matProjection, triClipped[n].t[2]);
					triProjected.colour = triClipped[n].colour;
					triProjected.symbol = triClipped[n].symbol;
					triProjected.tx[0] = triClipped[n].tx[0];
					triProjected.tx[1] = triClipped[n].tx[1];
					triProjected.tx[2] = triClipped[n].tx[2];

					// Scaling to view
					triProjected.t[0] = VecsDivide(triProjected.t[0], triProjected.t[0].w);
					triProjected.t[1] = VecsDivide(triProjected.t[1], triProjected.t[1].w);
					triProjected.t[2] = VecsDivide(triProjected.t[2], triProjected.t[2].w);

					//// X/Y are inverted so put them back
					//triProjected.t[0].x *= -1.0f;
					//triProjected.t[0].y *= -1.0f;
					//triProjected.t[1].x *= -1.0f;
					//triProjected.t[1].y *= -1.0f;
					//triProjected.t[2].x *= -1.0f;
					//triProjected.t[2].y *= -1.0f;

					// Offset vertices to visible normalised view
					Vec3D vOffsetView = { 1,1,0 };
					triProjected.t[0] = VecsAdd(triProjected.t[0], vOffsetView);
					triProjected.t[1] = VecsAdd(triProjected.t[1], vOffsetView);
					triProjected.t[2] = VecsAdd(triProjected.t[2], vOffsetView);

					// Scaling
					triProjected.t[0].x *= 0.5f * (float)ScreenWidth();
					triProjected.t[0].y *= 0.5f * (float)ScreenHeight();
					triProjected.t[1].x *= 0.5f * (float)ScreenWidth();
					triProjected.t[1].y *= 0.5f * (float)ScreenHeight();
					triProjected.t[2].x *= 0.5f * (float)ScreenWidth();
					triProjected.t[2].y *= 0.5f * (float)ScreenHeight();

					// Store triangles for sorting
					vecTrianglesToRaster.push_back(triProjected);
				}
			}
		}

		// Sort triangles from back to front
		sort(vecTrianglesToRaster.begin(), vecTrianglesToRaster.end(), [](Triangle& t1, Triangle& t2) {
				float z1 = (t1.t[0].z + t1.t[1].z + t1.t[2].z) / 3.0f;
				float z2 = (t2.t[0].z + t2.t[1].z + t2.t[2].z) / 3.0f;
				return z1 > z2;
			}
		);

		// Clear Screen
		Fill(0, 0, ScreenWidth(), ScreenHeight(), PIXEL_SOLID, FG_BLACK);

		for (auto& triToRaster : vecTrianglesToRaster) {
			// Clip triangles against all 4 screen edges
			Triangle triClipped[2];
			std::list<Triangle> listTriangles;
			listTriangles.push_back(triToRaster);
			int nNewTriangles = 1;

			for (int p = 0; p < 4; p++) {
				int nTrisToAdd = 0;
				while (nNewTriangles > 0) {
					// Take triangle from front of queue
					Triangle triTest = listTriangles.front();
					listTriangles.pop_front();
					nNewTriangles--;

					// Clip it against a plane. All triangles after a plane clip are guaranteed to lie on the inside of the plane
					switch (p) {
					case 0:	
						nTrisToAdd = TriangleClipAgainstPlane({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, triTest, triClipped[0], triClipped[1]);
						break;
					case 1:	
						nTrisToAdd = TriangleClipAgainstPlane({ 0.0f, (float)ScreenHeight() - 1, 0.0f }, { 0.0f, -1.0f, 0.0f }, triTest, triClipped[0], triClipped[1]);
						break;
					case 2:	
						nTrisToAdd = TriangleClipAgainstPlane({ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, triTest, triClipped[0], triClipped[1]);
						break;
					case 3:	
						nTrisToAdd = TriangleClipAgainstPlane({ (float)ScreenWidth() - 1, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }, triTest, triClipped[0], triClipped[1]);
						break;
					}

					// Add these new triangles to the back of the queue for subsequent clipping against next planes
					for (int w = 0; w < nTrisToAdd; w++)
						listTriangles.push_back(triClipped[w]);
				}
				nNewTriangles = listTriangles.size();
			}

			for (auto& tri : listTriangles) {
				//FillTriangle(tri.t[0].x, tri.t[0].y, tri.t[1].x, tri.t[1].y, tri.t[2].x, tri.t[2].y, tri.symbol, tri.colour);
				DrawTriangle(tri.t[0].x, tri.t[0].y, tri.t[1].x, tri.t[1].y, tri.t[2].x, tri.t[2].y, PIXEL_SOLID, FG_WHITE);
			}
		}

		return true;
	}
};

//

int main() {
	GraphicsEngine3D demo;
	if (demo.ConstructConsole(256, 240, 4, 4)) {
		demo.Start();
	}

	return 0;
}