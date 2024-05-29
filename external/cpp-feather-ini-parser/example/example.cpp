#include <iostream>
#include <cstring>
#include <stdint.h>
#include "../INI.h"

//#define FINI_WIDE_SUPPORT

using namespace std;

void centerString(string str); //Printing to console
std::string getStringFromFile(const std::string& path); //Source for data loading from memory.

int main()
{
   INI::PARSE_FLAGS = INI::PARSE_COMMENTS_ALL | INI::PARSE_COMMENTS_SLASH | INI::PARSE_COMMENTS_HASH;
   INI::SAVE_FLAGS = INI::SAVE_PRUNE | INI::SAVE_PADDING_SECTIONS | INI::SAVE_SPACE_SECTIONS | INI::SAVE_SPACE_KEYS | INI::SAVE_TAB_KEYS | INI::SAVE_SEMICOLON_KEYS;

   INI ini2("file.ini", true);  // Assign ini file and parse
   INI ini(ini2); // Clone
   //ini.parse(PARSE_COMMENTS_ALL | PARSE_COMMENTS_SLASH | PARSE_COMMENTS_HASH);

   ///Manipulate and access contents
   centerString("########## Access & Manipulate Contents ##########");

   //Common usage
   ini.create("Section 0");
   ini.select("Section 0");
   ini.set("Key2", "Value");
   ini.get("Key1", "DefaultValue");

   ini.set("Section 0", "Key1", std::to_string(11.12));
   cout << ini.getAs<float>("Section 2", "Key1") << endl;

   ini["Section 0"]["Key1"] = "Changed";

   // Save with formatting
   ini.save("test.ini");

   ///Iterate through sections and keys for both C++11 and C++98
   centerString("########## Iterate Contents ##########");

   // Loop through sections, keys and values
   for(auto i: ini.sections) {
      cout << "[" << i.first << "]" << endl;

      //for(auto j = i.second->begin(); j != i.second->end(); j++)
      for(auto j: *i.second) {
         cout << "  " << j.first << "=" << j.second << endl;
      }
   }


///Wide char support example (please define FINI_WIDE_SUPPORT in project) - OLD
   /*
   wcout << ini_w[L"Section2"][L"Key1"] << endl;
   */

///Load from memory - OLD
   /*std::string str = getStringFromFile("config/test.ini"); //Allows us to tap into a source for the purpose of this example

   INI ini_mem((void*)str.c_str(), str.size(), true); //This is the line which parses data from memory

///Merge contents and keep values
   INI inid("file.ini", true);
   INI inis("merge.ini", true);
   inid.merge(inis, true);
   inid.save("merged.ini");

   return EXIT_SUCCESS;*/
}

void centerString(string str)
{
   const char* s = str.c_str();
   int l = strlen(s);
   int pos = (int)((80 - l) / 2);
   for(int i = 0; i < pos; i++)
   cout << " ";
   cout << s << endl;
}

std::string getStringFromFile(const std::string& path) {
  std::ostringstream buf;
  std::ifstream input (path.c_str());
  buf << input.rdbuf();
  return buf.str();
}
