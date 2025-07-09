#pragma once
#include "hta/CVector.h"
#include "GeometryInfo.h"

namespace m3d
{
	struct DecalData
	{
		CVector pos;
		CVector normal;
		CVector tangent;
		m3d::GeometryInfo toPutOn;
	};
}