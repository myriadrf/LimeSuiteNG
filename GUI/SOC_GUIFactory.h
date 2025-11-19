#ifndef __BASE_FACTORY_H
#define __BASE_FACTORY_H

#include <string>
#include <unordered_map>

class ISOCPanel;

typedef ISOCPanel* (*panelConstructorFunc)(wxWindow* parent, wxWindowID id);

class SOC_GUIFactory
{
  public:
    static bool Register(const std::string_view name, panelConstructorFunc createMethod);
    static ISOCPanel* make(const std::string_view name, wxWindow* parent, wxWindowID id);

  protected:
    static std::unordered_map<std::string, panelConstructorFunc> RegisteredNames;
};

template<class FactoryT, panelConstructorFunc CreateMethod> bool RegisterToFactory(std::string_view name)
{
    return FactoryT::Register(name, CreateMethod);
}

#endif