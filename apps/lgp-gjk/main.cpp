#include <KOMO/komo.h>

#include <Kin/taskMap_GJK.h>
#include <shape_pair_fcl.h>

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

#include <math_utility.h>


using namespace std;
using namespace fcl;

//===========================================================================

void move()
{
  rai::KinematicWorld W;
  W.init( "model.g" );

  W.watch();

  rai::Shape *s1 = W.getShapeByName( "obstacle" );
  rai::Shape *s2 = W.getShapeByName( "block" );
  CHECK(s1 && s2,"");
  CHECK(s1->sscCore.V.N,"");
  CHECK(s2->sscCore.V.N,"");
  rai::Vector p1, p2, e1, e2;
  GJK_point_type pt1, pt2;

  GJK_sqrDistance(s1->sscCore, s2->sscCore, s1->X, s2->X, p1, p2, e1, e2, pt1, pt2);

  arr y1, J1, y2, J2;
  arr J, u, v;

  W.kinematicsPos(y1, (&J?J1:NoArr), s1->body, s1->body->X.rot/(p1-s1->body->X.pos));
  W.kinematicsPos(y2, (&J?J2:NoArr), s2->body, s2->body->X.rot/(p2-s2->body->X.pos));
  u = y1 - y2;

  //reduce by radii
  double l2=sumOfSqr(v), l=sqrt(l2);
  double fac = (l-s1->size(3)-s2->size(3))/l;
//  if(&J){
//    arr d_fac = (1.-(l-s1->size(3)-s2->size(3))/l)/l2 *(~v)*J;
//    J = J*fac + v*d_fac;
//  }
  v = u * fac;

  std::cout << "u norm:" << sqrt( norm2( u ) ) << std::endl;
  std::cout << "u:" << u << std::endl;

  std::cout << "v norm:" << sqrt( norm2( v ) ) << std::endl;
  std::cout << "v:" << v << std::endl;

  rai::wait( true );
}

CollisionObject * createObjectModel( rai::Shape * s )
{
  CHECK( s, "" );
  CHECK( s->mesh.V.N, "" );

  /// 1 - set mesh triangles and vertice indices
  std::vector<Vec3f> vertices;
  std::vector<Triangle> triangles;

  // code to set the vertices and triangles
  for( uint i = 0; i < s->mesh.V.d0; i++ )
  {
    vertices.push_back( Vec3f( s->mesh.V( i, 0 ), s->mesh.V( i, 1 ), s->mesh.V( i, 2 ) ) );
  }

  for( uint i = 0; i < s->mesh.T.d0; i++ )
  {
    triangles.push_back( Triangle( s->mesh.T( i, 0 ), s->mesh.T( i, 1 ), s->mesh.T( i, 2 ) ) );
  }

  auto model = std::make_shared< BVHModel<OBBRSS> >();
  // add the mesh data into the BVHModel structure
  model->beginModel();
  model->addSubModel(vertices, triangles);
  model->endModel();

  /// 2 - Set transforms
  Matrix3f R;
  Vec3f T;
  // code for setting R and T
  T = Vec3f( s->X.pos.x,  s->X.pos.y,  s->X.pos.z );

  auto m = s->X.rot.getMatrix();
  R = Matrix3f( m.m00, m.m01, m.m02,
                m.m10, m.m11, m.m12,
                m.m20, m.m21, m.m22 );

  // transform is configured according to R and T
  Transform3f pose( R, T );

  /// 3 - Combine them together
  CollisionObject * obj = new CollisionObject( model, pose );

  return obj;
}

void move_fcl()
{
  rai::KinematicWorld W;
  W.init( "model.g" );

  W.watch();

  rai::Shape *s1 = W.getShapeByName( "obstacle" );
  rai::Shape *s2 = W.getShapeByName( "block" );
  CHECK(s1 && s2,"");
  CHECK(s1->mesh.V.N,"");
  CHECK(s2->mesh.V.N,"");

  auto m1 = createObjectModel( s1 );
  auto m2 = createObjectModel( s2 );

  // set the distance request structure, here we just use the default setting
  DistanceRequest request;
  request.enable_nearest_points = true;
  // result will be returned via the collision result structure
  DistanceResult result;
  // perform distance test
  distance(m1, m2, request, result);

  auto p1 = result.nearest_points[ 0 ];
  auto p2 = result.nearest_points[ 1 ];
  auto v = p1 - p2;

  std::cout << "min distance:" << result.min_distance << std::endl;
  std::cout << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
  std::cout << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;


  std::cout << "v norm:" << v.norm() << std::endl;
  std::cout << "v:" << v << std::endl;


  rai::wait( true );
}

void move_fcl_tm()
{
  rai::KinematicWorld W;
  W.init( "model.g" );

  W.watch();

  auto tm = new ShapePairFCL( W, "obstacle", "block" );

  arr y, J;
  tm->phi( y, J, W, 0 );

  rai::wait( true );
}

//=========================================================================

int main(int argc,char** argv)
{
  rai::initCmdLine(argc,argv);

  //move_fcl();
  move_fcl_tm();

  return 0;
}
