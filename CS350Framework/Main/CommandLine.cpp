///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2017, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

#include "CommandLine.hpp"

CommandLine* CommandLine::GetInstance()
{
  static CommandLine mCommandLine;
  return &mCommandLine;
}

void CommandLine::Parse(int argc, char* argv[], ArgumentMap& defaults)
{
  std::string previousArg = "";
  for(int i = 3; i < argc; ++i)
  {
    std::string currentArg = argv[i];
    if(currentArg.empty())
      continue;

    // If this argument starts with a '-' then it is a flag
    if(currentArg[0] == '-')
    {
      // If there was a previous argument, that means we had a '-something' that had no value specified.
      // Look-up the default value for this argument if it existed and add it to our argument results
      if(!previousArg.empty())
      {
        std::string value;
        auto it = defaults.find(previousArg);
        if(it != defaults.end())
          value = it->second;
        mParsedArguments[previousArg] = value;
        mArgumentList.push_back(previousArg);
      }
      // Push on the current argument
      previousArg = currentArg;
    }
    // Otherwise, this is an argument value for the previous argument flag
    else
    {
      if(!previousArg.empty())
      {
        mParsedArguments[previousArg] = currentArg;
        mArgumentList.push_back(previousArg);
        previousArg.clear();
      }
    }
  }

  // If we ended with a flag with no value then lookup the defaults
  if(!previousArg.empty())
  {
    std::string value;
    auto it = defaults.find(previousArg);
    if(it != defaults.end())
      value = it->second;
    mParsedArguments[previousArg] = value;
    mArgumentList.push_back(previousArg);
  }
}

bool CommandLine::ContainsArgument(const std::string& argumentName, std::string* argumentValue)
{
  auto it = mParsedArguments.find(argumentName);
  if(it == mParsedArguments.end())
    return false;

  if(argumentValue != nullptr)
    *argumentValue = it->second;
  return true;
}
