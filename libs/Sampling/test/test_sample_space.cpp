#include <sample_space.h>
#include <iostream>
#include <gtest/gtest.h>

TEST(SampleSpace, SampleBounds)
{
  SampleSpace<2> space(std::array<std::pair<double, double>, 2>{std::pair<double, double>{-1.0, 1.0}, std::pair<double, double>{-1.0, 1.0}});

  for(auto i = 0; i < 100; ++i)
  {
    auto s = space.sample();

    //
    //std::cout << s[0] << " " << s[1] << std::endl;
    //

    EXPECT_GE(s[0], -1.0);
    EXPECT_LE(s[0], 1.0);

    EXPECT_GE(s[1], -1.0);
    EXPECT_LE(s[1], 1.0);
  }
}


//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
