#ifndef _CUSTOM_KEY_HANDLER_
#define _CUSTOM_KEY_HANDLER_

#include "CustomScene.h"

class CustomKeyHandler : public osgGA::GUIEventHandler {
  CustomScene &scene;
 public:
 CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
  bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

#endif
