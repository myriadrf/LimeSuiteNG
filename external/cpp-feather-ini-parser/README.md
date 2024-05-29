feather-ini-parser
==================

Intuitive, fast, lightweight, header, portable INI parser for ANSI C++.

```
INI<> ini("filename.ini", true); // Open and parse
ini.get("section", "key", "value", "default"); // Get section -> key -> value, return "default" if not found
ini.set("section", "key", "value"); // Set section -> key -> value
ini.save(); // Save over initial file

for(auto i: ini.sections) // Loop through all sections
{
   String section = i.first;

   for(auto j: *i.second) // Loop through all key/values
      String key = j.first, value = j.second;
}
```

## Methods

Statement     | Return Type
------------- | -------------
ini(filename, doParse)|constructor
ini(data, dataSize, doParse)|constructor
ini.parse()|bool
ini.merge(other INI, retainValues)|void
ini.create(section)|bool
ini.select(section)|bool
ini.set(key, value)|bool
ini.get(key, dvalue = value_t())|dvalue_t
ini.save(filename = "")|bool
ini.clear()|bool
ini[section][key]|value_t&
ini[section]|keys_t&

## Example
```
// Please view the complete list of examplex in 'example/example.cpp'

#include <iostream>
#include <String>
#include "INI.h"

using namespace std;
```
...
```
INI<> ini("filename.ini", true); // Load file and parse

ini.create("section1"); //Create a section and select it (into the active context)
ini.set("key", "value");
// Set equivelant
ini.set("section1", "key", "value");
// Set equivelant (non-safe, performance)
ini["section1"]["key"] = "value";

cout << ini.get("keynumeric", "default") << endl;
// Get equivelant
cout << ini.get("section1", "keynumeric", "default") << endl;

ini.save();

// Loop through all sections and keys (CPP11)
for(auto i: ini.sections)
{
   String section = i.first;
   cout << "[" << section << "]" << endl;

   for(auto j: *i.second)
   {
      String key = j.first, value = j.second;
      cout << " " << key << "=" << value << endl;
   }
}
```

## More
Please see the example .cpp file and Code::Blocks .cbp project for a compilable GCC and VSC++ example. Further examples include enabling wide char support.
