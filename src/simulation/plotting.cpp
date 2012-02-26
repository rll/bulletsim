#include "plotting.h"
#include "util.h"
#include <osg/PointSprite>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/BlendFunc>
#include <osg/ShapeDrawable>
#include <boost/foreach.hpp>

using namespace std;
using namespace util;

// based on galaxy example in osg docs

void PlotObject::setDefaultColor(float r, float g, float b, float a) {
  m_defaultColor = osg::Vec4(r,g,b,a);
}

void PlotObject::clear() {
  m_geom->getPrimitiveSetList().clear();
  osg::ref_ptr<osg::Vec3Array> osgPts = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array;
  m_geom->setVertexArray(osgPts);
  m_geom->setColorArray(osgCols);
}

PlotPoints::PlotPoints(float size) {
  m_geode = new osg::Geode();
  m_geom = new osg::Geometry();
  m_geom->setDataVariance(osg::Object::DYNAMIC);
  m_geode->addDrawable(m_geom);
  setDefaultColor(1,1,1,1);

  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
  osg::Point *point = new osg::Point();

  //  m_stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
  stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  stateset->setAttributeAndModes(blendFunc);
  stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

  point->setSize(size);
  stateset->setAttribute(point);
  m_geode->setStateSet(stateset);
}

void PlotPoints::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
  int nPts = osgPts->getNumElements();
  m_geom->setVertexArray(osgPts);
  m_geom->setColorArray(osgCols);
  m_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  m_geom->getPrimitiveSetList().clear();
  m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,nPts));
}

void PlotPoints::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts) {
  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(osgPts->size());
  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
  setPoints(osgPts, osgCols);
}

void PlotPoints::forceTransparency(float a) {
  osg::Vec4Array &colors = (osg::Vec4Array&) *m_geom->getColorArray();
  for (int i = 0; i < colors.size(); ++i) {
    osg::Vec4 c = colors[i];
    colors[i] = osg::Vec4(c.r(), c.g(), c.b(), a);
  }
}

void PlotPoints::setPoints(const vector<btVector3>& pts, const vector<btVector4>& cols) {
  setPoints(toVec3Array(pts), toVec4Array(cols));
}
void PlotPoints::setPoints(const vector<btVector3>& pts) {
  setPoints(toVec3Array(pts));
}

PlotLines::PlotLines(float width) {
  setDefaultColor(1,1,1,1);

  m_geode = new osg::Geode();
  m_geom = new osg::Geometry();
  m_geom->setDataVariance(osg::Object::DYNAMIC);
  m_geode->addDrawable(m_geom);

  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
  osg::LineWidth *linewidth = new osg::LineWidth();
  linewidth->setWidth(width);
  //m_stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
  stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  stateset->setAttribute(linewidth);

  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  stateset->setAttributeAndModes(blendFunc);
  stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

  m_geode->setStateSet(stateset);
}

void PlotLines::setPoints(const vector<btVector3>& pts, const vector<btVector4>& cols) {
  setPoints(toVec3Array(pts),  toVec4Array(cols));
}

void PlotLines::setPoints(const vector<btVector3>& pts) {
  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(pts.size());
  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
  setPoints(toVec3Array(pts),  osgCols);
}


void PlotLines::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts, const osg::ref_ptr<osg::Vec4Array>& osgCols) {
  int nPts = osgPts->getNumElements();
  m_geom->setColorArray(osgCols);
  m_geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
  m_geom->setVertexArray(osgPts);
  m_geom->getPrimitiveSetList().clear();
  m_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,nPts));
}

void PlotLines::setPoints(const osg::ref_ptr<osg::Vec3Array>& osgPts) {
  osg::ref_ptr<osg::Vec4Array> osgCols = new osg::Vec4Array(osgPts->size());
  BOOST_FOREACH(osg::Vec4& col, *osgCols) col = m_defaultColor;
  setPoints(osgPts, osgCols);
}

PlotSpheres::PlotSpheres() {
  m_geode = new osg::Geode();    
  m_nDrawables = 0;

  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
  //stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  stateset->setAttributeAndModes(blendFunc);
  stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

  m_geode->setStateSet(stateset);

};

void PlotSpheres::clear() {
  m_geode->removeDrawables(0,m_nDrawables);
}

void PlotSpheres::plot(const osg::ref_ptr<osg::Vec3Array>& centers, const osg::ref_ptr<osg::Vec4Array>& cols, const vector<float>& radii) {
  m_geode->removeDrawables(0,m_nDrawables);
  m_nDrawables = centers->size();
  for (int i=0; i < centers->size(); i++) {
    osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(0.25f);
    osg::Sphere* sphere = new osg::Sphere( centers->at(i), radii.at(i));
    osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere,hints);
    sphereDrawable->setColor(cols->at(i));
    m_geode->addDrawable(sphereDrawable);
  }
}

