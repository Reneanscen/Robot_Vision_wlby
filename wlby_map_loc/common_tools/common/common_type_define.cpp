#include "common/common_type_define.h"

namespace wlby{

    int SplitString(const std::string &input, const std::string &delimiter, std::vector<std::string> &results)
    {
        int iPos = 0;
        int newPos = -1;
        int sizeS2 = (int)delimiter.size();
        int isize = (int)input.size();
        results.clear();

        if ((isize == 0) || (sizeS2 == 0))
        {
            return 0;
        }

        std::vector<int> positions;

        int numFound = 0;

        while ((newPos = input.find(delimiter, iPos)) > 0)
        {
            positions.push_back(newPos);
            iPos = newPos + sizeS2;
            numFound++;
        }

        if (numFound == 0)
        {
            if (input.size() > 0)
            {
                results.push_back(input);
            }
            return 1;
        }

        if (positions.back() != isize)
        {
            positions.push_back(isize);
        }

        int offset = 0;
        std::string s("");

        for (int i = 0; i < (int)positions.size(); ++i)
        {

            s = input.substr(offset, positions[i] - offset);

            offset = positions[i] + sizeS2;

            bool all_space = std::all_of(s.begin(), s.end(), [](unsigned char c)
                                         { return std::isspace(c); });

            if (s.length() > 0 && !all_space)
            {
                results.push_back(s);
            }
        }
        return numFound;
    }
}