"""
Doxygen to Sphinx page converter
================================

Python module performs quick auto parsing of the regular doxygen pages (with @page tag) and generates 
the equivalent sphinx page files (.rst files) with the required Sphinx Breathe plugin directives. This 
module can be run in reverse mode to delete all the generated manual pages (.rst files). See supported 
flags for more info.

Usage:
   ``python dox_converter.py [override-root-path] [optional-flags]``

This module will auto scan each subdirectory starting from the root directory (by default that is ``./doxygen``) and will
generate .rst files equivalent to all the .dox files. To override the root directory, pass the root directory as a command 
line argument when calling the module.

Supported flags:

* `--del` - Runs dox_converter in reverse. Deletes all the .rst manual pages that were converted from doxygen manual pages.

This module does not support the auto generation of index files used to organize navigation between pages.
Users are responsible for setting up the correct index tree. Module will ignore files that have the following 
names: ``mainpage.dox, index.dox``. Therefore, it is recommended to use these file names for doxygen index pages. 
If index page naming is ignored, the module can produce undesired results.

"""

from pathlib import Path
import sys

reverse = False                  # Dox converter in reverse mode: deletes all of the generated manual pages in .rst format.
root_path = Path("./developer/doxygen/")   # Default root path
dox_suffix = ".dox"              # Doxygen suffix indentifier
rst_suffix = ".rst"              # Sphinx file suffix indentifier
dir_tree = []                    # The directory tree view from the root path 

index_indicators = ["mainpage", "index"]        # File names that indicate index file with toc tree
ignore_dirs = ["output", "api_member_list", "images"]   # Directories to ignore


def update_dir_tree(start_dir, ignore_list = ignore_dirs):
   """ 
   Creates a directory tree starting from the root directory.

   Starting from the start_dir, recursively scans through the directories and adds 
   them to the tree view, which is stored in dir_tree list.
    
   """
   dir_tree.append(start_dir)
   for dir_item in start_dir.iterdir():
      if dir_item.is_dir():
         
         # skip subdir if it is in ignore list
         if dir_item.name in ignore_list:
            continue
         update_dir_tree(dir_item)
      else:
         continue

def get_files(start_dir, suffix):
   """Gets the list of all files in specified diretory."""

   files = []
   for dir_item in start_dir.iterdir():
      if dir_item.is_file() and dir_item.suffix == suffix:
         files.append(dir_item)
      else:
         continue

   return files

def convert_page(file):
   """Parses doxygen page title line elements and applies rst page template with breathe directives."""

   rst_path = file.with_suffix(".rst")
   rst = rst_path.open("w")
   with file.open("r") as dox:
      dox.readline()
      header = dox.readline()
   header = header.strip()
   header_elements = header.split(maxsplit=3)
   rst.write(f"{header_elements[3]}\n{"=" * len(header_elements[3])}\n\n")
   rst.write(f".. doxygenpage:: {header_elements[2]}\n\t:content-only:")
   rst.close()

def convert_pages(files):
   """Generates sphinx compatible rst manual pages with breathe directives from doxygen pages."""

   for file in files:
      if file.stem not in index_indicators:
         convert_page(file)

def remove_pages(files):
   """Removes sphinx compatible rst manual pages."""

   for file in files:
      if file.stem not in index_indicators:
         file.unlink()

def print_tree():
   """Prints directory tree with contents of each directory."""

   for dir in dir_tree:
      print(f"{dir} contents:")
      for item in dir.iterdir():
         print(f"\t{item.name}")
      print("\n")
         
      
########################################
#  Script start
########################################

if len(sys.argv) > 1:
   for arg in sys.argv[1:]:
      if arg == "--del":
         reverse = True
      else:
         root_path = Path(f"{arg}")
         print(f"Doxygen page converter: Switching to new root path - {arg}")

update_dir_tree(root_path)
# print_tree()

# Delete generated rst manual pages
if reverse:
   for dir in dir_tree:
      files = get_files(dir, rst_suffix)
      remove_pages(files)

# Generate regular rst pages from regular doxygen pages
else:
   for dir in dir_tree:
      files = get_files(dir, dox_suffix)
      convert_pages(files)
   
# print_tree()
