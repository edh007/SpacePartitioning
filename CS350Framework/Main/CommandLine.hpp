///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2017, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

/// Helper class to parse command line arguments into a map for easy checking
class CommandLine
{
public:
  typedef std::unordered_map<std::string, std::string> ArgumentMap;
  typedef std::vector<std::string> ArgumentList;

  static CommandLine* GetInstance();

  /// Parse the given command line. Use the provided default arguments map to fill in any missing arguments.
  void Parse(int argc, char* argv[], ArgumentMap& defaults);

  /// Find the argument by name. Fill out the value if the second parameter is given.
  bool ContainsArgument(const std::string& argumentName, std::string* argumentValue = nullptr);

  ArgumentMap mParsedArguments;
  /// The arguments in the order received (values are not include, they can be looked up in the map)
  ArgumentList mArgumentList;
};
