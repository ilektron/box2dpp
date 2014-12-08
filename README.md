Efforts in improving Box2D library

- x Create b2 or b2d11 namespace
- x clang-format
- x clang-modernize
  - ranged for loops
  - nullptr
  - etc
- Better object create (utilizing move semantics?)
- Better scope
  - only private data members
  - get/set methods
- Examine allocators
- Switch to vectors where applicable
- Initializer lists for major objects
- enum naming
- gtest
- Profiling

ofxBox2DBasic using box2d11
   Use cmake for openFrameworks
