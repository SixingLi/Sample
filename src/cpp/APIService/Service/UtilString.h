#pragma once

#include <sstream>
#include <string>
#include <regex>


class UtilString
{
public:
    template<typename T>
    static std::string ToString(const T& t)
    {
        //Do not use std::move which will kill performance for large amount of calls
        return std::to_string(t);
    }

    template<typename T>
    static T FromString(const std::string& str)
    {
        std::stringstream ss;
        ss << str;
        T output;
        ss >> output;
        return output;
    }

    static std::vector<std::string> split(const std::string& input, const std::string& separator)
    {
        std::vector<std::string> vec;
        if (input.empty())
            return vec;
        std::regex re(separator);
        std::sregex_token_iterator p(input.begin(), input.end(), re, -1);
        std::sregex_token_iterator end;
        while (p != end)
            vec.emplace_back(*p++);
        return vec;
    }

    template<typename T>
    static std::vector<T> ListFromString(const std::string& str, const std::string& separator)
    {
        std::vector<T> list;
        std::vector<std::string> strList = split(str, separator);
        for (auto& item : strList)
        {
            T v = FromString<T>(item);
            list.push_back(v);
        }
        return list;
    }

    static std::string ListToString(const std::vector<double>& listOfData, const std::string& separator)
    {
        std::string ret;
        for (size_t i = 0; i < listOfData.size(); i++)
        {
            auto& item = listOfData[i];
            std::string str = UtilString::ToString(item);
            ret += str;
            if (i < listOfData.size() - 1)
            {
                ret += separator;
            }
        }
        return std::move(ret);
    }

    static std::string ListToString(const std::vector<long>& listOfData, const std::string& separator)
    {
        std::string ret;
        for (size_t i = 0; i < listOfData.size(); i++)
        {
            auto& item = listOfData[i];
            std::string str = UtilString::ToString(item);
            ret += str;
            if (i < listOfData.size() - 1)
            {
                ret += separator;
            }
        }
        return std::move(ret);
    }
};