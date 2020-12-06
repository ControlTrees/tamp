#include <nearest_neighbor.h>
#include <gtest/gtest.h>
#include <iostream>

// see examples https://www.geeksforgeeks.org/k-dimensional-tree/
// 0
TEST(KDTree, AddFirstPoint)
{
  KDTree<2> tree({3.0, 6.0});

  auto state = tree.root()->state;
  EXPECT_EQ(tree.root()->splitting_value, 3.0);
  EXPECT_EQ(state[0], 3.0);
  EXPECT_EQ(state[1], 6.0);
}

// 1
TEST(KDTree, AddSecondPointLeft)
{
  KDTree<2> tree({3.0, 6.0});

  tree.add_node({2.0, 7.0}, 1);

  auto node = tree.root()->left;
  auto state = node->state;
  EXPECT_EQ(state[0], 2.0);
  EXPECT_EQ(state[1], 7.0);
  EXPECT_EQ(node->axis, 1);
}

TEST(KDTree, AddSecondPointRight)
{
  KDTree<2> tree({3.0, 6.0});

  tree.add_node({17.0, 15.0}, 1);

  auto node = tree.root()->right;
  auto state = node->state;
  EXPECT_EQ(state[0], 17.0);
  EXPECT_EQ(state[1], 15.0);
  EXPECT_EQ(node->axis, 1);
}

// 3
TEST(KDTree, AddThirdLevelPoints)
{
  KDTree<2> tree({3.0, 6.0});

  tree.add_node({17.0, 15.0}, 1);
  tree.add_node({13.0, 15.0}, 2);
  tree.add_node({6.0, 12.0}, 3);
  tree.add_node({9.0, 1.0}, 4);
  tree.add_node({2.0, 7.0}, 5);
  tree.add_node({10.0, 19.0}, 6);

  // BFS left right
  {
  auto node = tree.root()->left;
  auto state = node->state;
  EXPECT_EQ(state[0], 2.0);
  EXPECT_EQ(state[1], 7.0);
  EXPECT_EQ(node->axis, 1);
  }

  {
  auto node = tree.root()->right;
  auto state = node->state;
  EXPECT_EQ(state[0], 17.0);
  EXPECT_EQ(state[1], 15.0);
  EXPECT_EQ(node->axis, 1);
  }

  {
  auto node = tree.root()->right->left;
  auto state = node->state;
  EXPECT_EQ(state[0], 6.0);
  EXPECT_EQ(state[1], 12.0);
  EXPECT_EQ(node->axis, 0);
  }

  {
  auto node = tree.root()->right->right;
  auto state = node->state;
  EXPECT_EQ(state[0], 13.0);
  EXPECT_EQ(state[1], 15.0);
  EXPECT_EQ(node->axis, 0);
  }

  {
  auto node = tree.root()->right->left->right;
  auto state = node->state;
  EXPECT_EQ(state[0], 9.0);
  EXPECT_EQ(state[1], 1.0);
  EXPECT_EQ(node->axis, 1);
  }

  {
  auto node = tree.root()->right->right->left;
  auto state = node->state;
  EXPECT_EQ(state[0], 10.0);
  EXPECT_EQ(state[1], 19.0);
  EXPECT_EQ(node->axis, 1);
  }
}

// nearest neighbors
TEST(KDTree, NearestNeighbors)
{
  KDTree<2> tree({3.0, 6.0});

  tree.add_node({17.0, 15.0}, 1);
  tree.add_node({13.0, 15.0}, 2);
  tree.add_node({6.0, 12.0}, 3);
  tree.add_node({9.0, 1.0}, 4);
  tree.add_node({2.0, 7.0}, 5);
  tree.add_node({10.0, 19.0}, 6);

  { // middle in the tree
  auto node = tree.nearest_neighbor({17.0, 15.0});
  auto state = node->state;

  EXPECT_EQ(state[0], 17.0);
  EXPECT_EQ(state[1], 15.0);
  }

  { // on a leaf
  auto node = tree.nearest_neighbor({9.1, 1.0});
  auto state = node->state;

  EXPECT_EQ(state[0], 9.0);
  EXPECT_EQ(state[1], 1.0);
  }

  { // on a leaf
  auto node = tree.nearest_neighbor({2.0, 8.0});
  auto state = node->state;

  EXPECT_EQ(state[0], 2.0);
  EXPECT_EQ(state[1], 7.0);
  }
}

// neighbors in radius
TEST(KDTree, NearestNeighborsInRadius)
{
  KDTree<2> tree({3.0, 6.0});

  tree.add_node({17.0, 15.0}, 1);
  tree.add_node({13.0, 15.0}, 2);
  tree.add_node({6.0, 12.0}, 3);
  tree.add_node({9.0, 1.0}, 4);
  tree.add_node({2.0, 7.0}, 5);
  tree.add_node({10.0, 19.0}, 6);

  {
  auto nodes = tree.radius_neighbors({12.0, 17.0}, 5.5);
  EXPECT_EQ(nodes.size(), 3);
  }

  {
  auto nodes = tree.radius_neighbors({10.0, 1.0}, 5.0);
  EXPECT_EQ(nodes.size(), 1);
  }

  {
  auto nodes = tree.radius_neighbors({3.0, 7.0}, 1.5);
  EXPECT_EQ(nodes.size(), 2);
  }

  {
  auto nodes = tree.radius_neighbors({10.0, 1.0}, 0.1);
  EXPECT_EQ(nodes.size(), 0);
  }
}

//
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
