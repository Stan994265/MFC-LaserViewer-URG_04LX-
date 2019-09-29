// Baocun.cpp: implementation of the CBaocun class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "laser.h"
#include "Baocun.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CBaocun::CBaocun()
{

}
CBaocun::CBaocun(CPoint m_ptPoint)
{
	this->m_ptPoint=m_ptPoint;
}

CBaocun::~CBaocun()
{

}
