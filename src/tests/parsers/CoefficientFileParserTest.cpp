#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "parsers/CoefficientFileParser.h"

#include <fstream>
#include <filesystem>
#include <string>
#include <string_view>

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

TEST(CoefficientFileParser, StandardOutputFile)
{
    constexpr std::string_view fileContents = R"""(/* ******************************************************************
   FILE:	rx2
   DESCRIPTION:	
   DATE:	
   REVISIONS:	
   ****************************************************************** */

	1.470962,
	1.203833,
	1.728385,
	0.165471,
	0.581988,
	0.380383,
	1.485763,
	0.278542,
	0.074313,
	1.760674,
	1.387982,
	1.452406,
	1.000000,
	0.366008,
	1.713828,
	1.365612,
	0.437727,
	1.226112,
	0.570177,
	0.308145,
	0.738365,
	0.405255,
	0.957701,
	1.430128,
	1.571398,
	1.124363,
	0.602954,
	0.087985,
	1.410993,
	0.167486,
	1.106784,
	0.144505,
	0.666616,
	1.205695,
	0.804224,
	0.694357,
	0.644124,
	1.283670,
	1.822108,
	0.338786
)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 1.470962,
        1.203833,
        1.728385,
        0.165471,
        0.581988,
        0.380383,
        1.485763,
        0.278542,
        0.074313,
        1.760674,
        1.387982,
        1.452406,
        1.000000,
        0.366008,
        1.713828,
        1.365612,
        0.437727,
        1.226112,
        0.570177,
        0.308145,
        0.738365,
        0.405255,
        0.957701,
        1.430128,
        1.571398,
        1.124363,
        0.602954,
        0.087985,
        1.410993,
        0.167486,
        1.106784,
        0.144505,
        0.666616,
        1.205695,
        0.804224,
        0.694357,
        0.644124,
        1.283670,
        1.822108,
        0.338786 };
    expectedResult.resize(40, 0);

    std::vector<double> coefficients(40, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 40);

    EXPECT_EQ(actualReturn, 40);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, SingleLineComments)
{
    constexpr std::string_view fileContents = R"""(0.111111 // 0.222222
    0.333333//0.444444
    0.555555 // Random characters as well
    0.777777
)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111, 0.333333, 0.555555, 0.777777 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 4);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, SinglelineCommentsMultilineCommentIsIgnored)
{
    constexpr std::string_view fileContents = R"""(0.111111 ///*0.222222 
    // /* 0.333333 
    0.444444)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111, 0.444444 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 2);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, SingleLineCommentFileEnd)
{
    constexpr std::string_view fileContents = R"""(0.111111 // 0.222222
    0.333333//0.444444
    0.555555// Random characters as well)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111, 0.333333, 0.555555 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 3);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, SingleLineCommentImmediateFileEnd)
{
    constexpr std::string_view fileContents = R"""(0.111111 //)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 1);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, MultilineCommentsSingleLine)
{
    constexpr std::string_view fileContents =
        R"""(0.111111/*0.222222*/0.333333/* 0.444444*/ 0.555555 /* 0.666666 */ 0.777777/*0.888888*/,0.99)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111, 0.333333, 0.555555, 0.777777, 0.99 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 5);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, MultilineCommentsMultiLine)
{
    constexpr std::string_view fileContents = R"""(0.111111/*0.222222
    */0.333333/* 
    0.444444*/ 0.555555 /* 
    0.666666 
    */ 0.777777)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111, 0.333333, 0.555555, 0.777777 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 4);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, MultilineCommentsNested)
{
    constexpr std::string_view fileContents = R"""(0.111111/*0.222222
    */0.333333/* /*
    0.444444*/*/ 0.555555 /*/**/*//*
    0.666666 
    */ 0.777777)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111, 0.333333, 0.555555, 0.777777 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 4);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, MultilineCommentsUnclosed)
{
    constexpr std::string_view fileContents = R"""(0.111111/* 0.222222
    0.333333)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 1);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, MultilineCommentsUnclosedImmediateEndOfFile)
{
    constexpr std::string_view fileContents = R"""(/*)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{};
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 0);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, CommasAreWhitespace)
{
    constexpr std::string_view fileContents = R"""(,,,,,0.111111,,,0.222222
 , , , 0.333333,,,,,,)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111, 0.222222, 0.333333 };
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 3);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, NoFilenameProvided)
{
    const std::filesystem::path fileName = ""s;

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);

    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, -3);
}

TEST(CoefficientFileParser, NonExistentFilenameProvided)
{
    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, -4);
}

TEST(CoefficientFileParser, EmptyFile)
{
    constexpr std::string_view fileContents = ""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{};
    expectedResult.resize(10, 0);

    std::vector<double> coefficients(10, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 10);

    EXPECT_EQ(actualReturn, 0);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, TooManyCoefficients)
{
    constexpr std::string_view fileContents = R"""(  0.111111 -0.222222)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.111111 };
    expectedResult.resize(1, 0);

    std::vector<double> coefficients(1, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 1);

    EXPECT_EQ(actualReturn, -5);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, SyntaxError)
{
    constexpr std::string_view fileContents = R"""(0.123456 These characters are not in a comment)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.123456 };
    expectedResult.resize(1, 0);

    std::vector<double> coefficients(1, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 1);

    EXPECT_EQ(actualReturn, -2);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, EndOfFileWithComma)
{
    constexpr std::string_view fileContents = R"""(0.123456,)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    std::ofstream outFile(fileName);
    outFile << fileContents;
    outFile.close();

    std::vector<double> expectedResult{ 0.123456 };
    expectedResult.resize(1, 0);

    std::vector<double> coefficients(1, 0);
    auto parser = CoefficientFileParser(fileName);
    int actualReturn = parser.getCoefficients(coefficients, 1);

    EXPECT_EQ(actualReturn, 1);
    EXPECT_THAT(coefficients, expectedResult);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, SaveFile)
{
    constexpr std::string_view fileContents = R"""(/* ******************************************************************
   FILE:	CoefficientFileParser_SaveFile.fir
   DESCRIPTION:	
   DATE:	
   REVISIONS:	
   ****************************************************************** */

	0.111111,
	-0.222222,
	0.333333)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName = testSuiteName + "_"s + testName + ".fir"s;

    const std::vector<double> input{ 0.111111, -0.222222, 0.333333 };
    auto parser = CoefficientFileParser(fileName);
    parser.saveToFile(input);

    std::ifstream in(fileName);

    std::string actualFileContents;
    std::getline(in, actualFileContents, '\0');
    in.close();

    EXPECT_EQ(actualFileContents, fileContents);

    std::filesystem::remove(fileName);
}

TEST(CoefficientFileParser, SaveFileInSubfolder)
{
    constexpr std::string_view fileContents = R"""(/* ******************************************************************
   FILE:	CoefficientFileParser_SaveFileInSubfolder.fir
   DESCRIPTION:	
   DATE:	
   REVISIONS:	
   ****************************************************************** */

	0.111111,
	-0.222222,
	0.333333)"""sv;

    const std::string testSuiteName = ::testing::UnitTest::GetInstance()->current_test_info()->test_suite_name();
    const std::string testName = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const std::filesystem::path fileName =
        "./"s + testSuiteName + "_"s + testName + ".fir"s; // Use this folder as the testing path.

    const std::vector<double> input{ 0.111111, -0.222222, 0.333333 };
    auto parser = CoefficientFileParser(fileName);
    parser.saveToFile(input);

    std::ifstream in(fileName);

    std::string actualFileContents;
    std::getline(in, actualFileContents, '\0');
    in.close();

    EXPECT_EQ(actualFileContents, fileContents);

    std::filesystem::remove(fileName);
}
