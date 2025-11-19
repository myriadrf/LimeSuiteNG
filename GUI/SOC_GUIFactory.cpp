#include "SOC_GUIFactory.h"

#include <utility>
#include <cstdio>

bool SOC_GUIFactory::Register(std::string_view name, panelConstructorFunc createMethod)
{
    const auto registeredPair = SOC_GUIFactory::RegisteredNames.find(std::string(name));
    if (registeredPair != SOC_GUIFactory::RegisteredNames.end())
    {
        std::cerr << "SOC_GUIFactory: class(" << name << ") already registered" << std::endl;
        return false;
    }

    // std::cout << "SOC GUI registered(" << name << ")\n";
    SOC_GUIFactory::RegisteredNames.insert(std::make_pair(std::string(name), createMethod));
    return true;
}

ISOCPanel* SOC_GUIFactory::make(std::string_view name, wxWindow* parent, wxWindowID id)
{
    auto registeredPair = SOC_GUIFactory::RegisteredNames.find(std::string(name));
    if (registeredPair == SOC_GUIFactory::RegisteredNames.end())
        return nullptr;

    return registeredPair->second(parent, id);
}

// initialise the registered names map
std::unordered_map<std::string, panelConstructorFunc> SOC_GUIFactory::RegisteredNames = {};