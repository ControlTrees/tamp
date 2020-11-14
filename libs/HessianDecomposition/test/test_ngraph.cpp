#include <HessianDecomposition/ngraph.h>
#include <Core/array.h>
#include <fstream>

#include <gtest/gtest.h>

using namespace std;

namespace NGraph
{
static void to_gv(const Graph & G, std::ostream & os)
{
  os << "graph G{\n";

  for (Graph::const_iterator p = G.begin(); p != G.end(); p++)
  {
      if (Graph::out_neighbors(p).size() == 0  &&
          Graph::in_neighbors(p).size() == 0)
      {
        os << Graph::node(p) << ";\n";
      }
  }
  for (Graph::const_iterator p = G.begin(); p != G.end(); p++)
  {
      const Graph::vertex_set &out = Graph::out_neighbors(p);
      Graph::vertex from = Graph::node(p);
      for (Graph::vertex_set::const_iterator q = out.begin();
                  q != out.end(); q++)
      {
          //if (from <= *q)
          os << from << "--" << *q << " ;\n";
      }
  }

  os << "}";
}

static void to_file(const Graph & G, const std::string & filepath, bool png=false)
{
  std::ofstream file;
  file.open(filepath, ios::out);

  to_gv(G, file);

  file.close();

  if(png)
  {
    std::string name_copy(filepath);
    const std::string ext( ".gv" );
    std::string new_name = name_copy.replace( name_copy.find( ext ), ext.length(), ".png" );

    std::stringstream ss;
    ss << "dot"   << " ";
    ss << "-Tpng" << " ";
    ss << "-o"    << " ";
    ss << new_name << " ";
    ss << filepath;

    system( ss.str().c_str() );
  }
}

static Graph from_hessian(const arr& H)
{
  NGraph::Graph G;

  for(auto i = 0; i < H.d0; ++i)
  {
    for(auto j = 0; j < i; ++j)
    {
      if(H(i, j))
      {
        G.insert_edge(i, j);

        std::cout << i << " " << j << std::endl;
      }
    }
  }

  return G;
}

}

static arr build_simple_decoupled()
{
  arr J = zeros(3, 4);
  J(0, 0) = 1;
  J(0, 2) = 1;
  J(1, 1) = 1;
  J(1, 3) = 1;
  J(2, 1) = 1;
  J(2, 3) = 1;

  //std::cout << J << std::endl;
  auto H = comp_At_A(J);

  return H;
}

static arr build_simple_light_coupling()
{
  arr J = zeros(5, 6);
  J(0, 0) = 1; J(0, 2) = 1;
  J(1, 2) = 1; J(1, 4) = 1;
  J(2, 1) = 1; J(2, 3) = 1;
  J(3, 3) = 1; J(3, 5) = 1;
  J(4, 2) = 1; J(4, 3) = 1;

  //std::cout << J << std::endl;
  auto H = comp_At_A(J);

  return H;
}


static arr build_medium_light_coupling()
{
  arr J = zeros(5, 12);
  J(0, 0) = 1;  J(0, 1) = 1; J(0, 4) = 1; J(0, 5) = 1;
  J(1, 4) = 1;  J(1, 5) = 1; J(1, 8) = 1; J(1, 9) = 1;
  J(2, 2) = 1;  J(2, 3) = 1; J(2, 6) = 1; J(2, 7) = 1;
  J(3, 6) = 1;  J(3, 7) = 1; J(3, 10) = 1; J(3, 11) = 1;
  J(4, 5) = 1;  J(4, 6) = 1;

  std::cout << J << std::endl;
  auto H = comp_At_A(J);

  return H;
}


TEST(Graph, InsertEdge)
{
  NGraph::Graph A;
  A.insert_edge(3,4);
  A.insert_edge(4,5);

  EXPECT_EQ(3, A.num_vertices());
  EXPECT_EQ(2, A.num_edges());
}

TEST(Graph, SaveToFile)
{
  NGraph::Graph B;
  B.insert_edge(5,2);
  B.insert_edge(3,4);
  B.insert_edge(4,5);

  NGraph::to_gv(B, std::cout);
  NGraph::to_file(B, "graph.gv", true);
}

TEST(Graph, RelationJacobianHessian)
{
  auto H = build_simple_decoupled();

  EXPECT_EQ(H.d0, 4);
}

TEST(Graph, BuildGraphOutOfHessianDecoupled)
{
  auto H = build_simple_decoupled();

  NGraph::Graph G = NGraph::from_hessian(H);

  NGraph::to_file(G, "simple_decoupled.gv", true);
}

TEST(Graph, BuildGraphOutOfHessianLightCoupling)
{
  auto H = build_simple_light_coupling();

  NGraph::Graph G = NGraph::from_hessian(H);

  NGraph::to_file(G, "build_simple_light_coupling.gv", true);
}

TEST(Graph, BuildGraphOutOfHessianMediumLightCoupling)
{
  auto H = build_medium_light_coupling();

  NGraph::Graph G = NGraph::from_hessian(H);

  NGraph::to_file(G, "build_medium_light_coupling.gv", true);
}

////////////////////////////////
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
