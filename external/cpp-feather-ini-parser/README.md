

feather-ini-parser
==================

Intuitive, fast, lightweight, header, portable INI parser for ANSI C++.

## Methods

Statement     | Return Type
------------- | -------------
**ini**(filename, doParse, parseFlags)|constructor
**ini**(ini)|copy constructor
**ini.parse**(parseFlags)|bool
**ini.save**(filename, saveFlags)|bool
**ini.set**(section, key, value)|bool
**ini.get**(section, key, def)|string
**ini.getAs**\<type\>(section, key, def = type())|type
ini.create(section)|bool
ini.select(section)|bool
ini.set(key, value)|bool
ini.get(key, def)|string
ini.getAs\<type\>(key, def = type())|type
ini.clear()|bool
ini[section][key]|string&
ini[section]|keys_t&

## Flags

Flag     | Effect
------------- | -------------
PARSE_COMMENTS_ALL| Parses comments // and #
PARSE_COMMENTS_SLASH| Parses comment //
PARSE_COMMENTS_HASH| Parses comment #
SAVE_PRUNE| Removes all empty sections/keys
SAVE_PADDING_SECTIONS| Appends a blank line under each section
SAVE_SPACE_SECTIONS| '[ Section ]' instead of '[Section]'
SAVE_SPACE_KEYS| 'key = value' instead of 'key=value'
SAVE_TAB_KEYS| '    key=value' instead of 'key=value'
SAVE_SEMICOLON_KEYS| 'key=value;' instead of 'key=value'

## Example

**Requires C++11:**
-std=c++11 **or** -std=c++14 **or** -std=c++20 **or** ...

```js
#include "INI.h"
//#define FINI_WIDE_SUPPORT
...

// Set global parsing/saving options
INI::PARSE_FLAGS = INI::PARSE_COMMENTS_ALL | INI::PARSE_COMMENTS_SLASH | INI::PARSE_COMMENTS_HASH;
INI::SAVE_FLAGS = INI::SAVE_PRUNE | INI::SAVE_PADDING_SECTIONS | INI::SAVE_SPACE_SECTIONS | INI::SAVE_SPACE_KEYS | INI::SAVE_TAB_KEYS | INI::SAVE_SEMICOLON_KEYS;

INI ini("file.ini", true);  // Assign ini file and parse

ini.create("Section 0");
ini.select("Section 0");
ini.set("Key2", "Value");
ini.get("Key1", "DefaultValue");

ini.set("Section 0", "Key1", std::to_string(11.12));
cout << ini.getAs<float>("Section 2", "Key1") << endl; // Return as float

ini["Section 0"]["Key1"] = "Changed";

// Save with formatting
ini.save("test.ini");

// Loop through sections, keys and values
for(auto i: ini.sections) {
   cout << "[" << i.first << "]" << endl;

   //for(auto j = i.second->begin(); j != i.second->end(); j++)
   for(auto j: *i.second) {
      cout << "  " << j.first << "=" << j.second << endl;
   }
}
```

## More
Please see the example .cpp file and Code::Blocks .cbp project for a compilable GCC and VSC++ example. Further examples include enabling wide char support.
