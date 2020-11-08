/*#include <shape_pair_fcl.h>

#include <Geo/mesh.h>
#include <Kin/proxy.h>
#include <Kin/frame.h>

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

using namespace fcl;

//-------ShapePairFCL----------------//

void ShapePairFCL::phi( arr& y, arr& J, const rai::KinematicWorld& G, int t )
{
  arr tmp_y = zeros( dim_phi( G ) );
  arr tmp_J = zeros( dim_phi( G ), G.q.N );

  for( rai::Proxy *p: G.proxies )
  {
    if((p->a==i_ && p->b==j_) || (p->a==j_ && p->b==i_))
    {
      if( p->d > 0 )
      {
        phiProxy( tmp_y, tmp_J, G, p );
        //phiFCL( tmp_y, tmp_J, G );
      }
      else
      {
        // collision already!
        //phiFCL( tmp_y, tmp_J, G );
      }
      break;
    }
  }

  // commit results
  y = tmp_y;
  if(&J) J = tmp_J;
}

void ShapePairFCL::phiProxy( arr& y, arr& J, const rai::KinematicWorld& G, rai::Proxy * p )
{
  rai::Shape *a = G.shapes(p->a);
  rai::Shape *b = G.shapes(p->b);

  auto arel=a->body->X.rot/(p->posA-a->body->X.pos);
  auto brel=b->body->X.rot/(p->posB-b->body->X.pos);

  arr posA;
  arr posB;
  arr JposA;
  arr JposB;
  G.kinematicsPos(posA, JposA, a->body, arel);
  G.kinematicsPos(posB, JposB, b->body, brel);

  double d   = norm2( posA - posB );
  arr JnormD = Jnorm( posA - posB );

  const double w = 10;
  y( 0 ) = w * ( - d + 0.05 );
  J = w * ( - JnormD * ( JposA - JposB ) );
}

void ShapePairFCL::phiFCL( arr& y, arr& J, const rai::KinematicWorld& G )
{
  rai::Shape *s1 = i_<0?NULL: G.shapes(i_);
  rai::Shape *s2 = j_<0?NULL: G.shapes(j_);
  CHECK(s1 && s2,"");

  auto m1 = createObjectModel( s1 );
  auto m2 = createObjectModel( s2 );

  // set the distance request structure, here we just use the default setting
  DistanceRequest request;
  request.enable_nearest_points = true;
  // result will be returned via the collision result structure
  DistanceResult result;
  // perform distance test
  distance(m1, m2, request, result);

  bool collision = result.min_distance == 0;

  auto p1 = result.nearest_points[ 0 ];
  auto p2 = result.nearest_points[ 1 ];

  rai::Vector pposA( p1[ 0 ], p1[ 1 ], p1[ 2 ] );
  rai::Vector pposB( p2[ 0 ], p2[ 1 ], p2[ 2 ] );

  /////////
  auto a = s1;
  auto b = s2;
  auto arel=a->body->X.rot/(pposA-a->body->X.pos);
  auto brel=b->body->X.rot/(pposB-b->body->X.pos);

  std::cout << "arel:" << arel << std::endl;
  std::cout << "brel:" << brel << std::endl;

  arr posA;
  arr posB;
  arr JposA;
  arr JposB;
  G.kinematicsPos(posA, JposA, a->body, arel);
  G.kinematicsPos(posB, JposB, b->body, brel);

  const double w = 10;

  auto v = posA - posB;
  double d = norm2( v );
  arr JnormD = Jnorm( v );

  std::cout << "collision:" << collision << std::endl;

  std::cout << "v:" << v << std::endl;
  std::cout << "angle:" << std::atan2( v( 1 ), v( 0 ) ) * 180 / 3.1415 << std::endl;

  if( collision )
  { 
    y( 0 ) = w * ( d + 0.05 );
    J = w * JnormD * ( JposA - JposB );
  }
  else
  {
    CHECK( fabs( d - result.min_distance ) < 0.00001, "uncoherent result in shape proximity computation" );

    y( 0 ) = w * ( -d + 0.05 );
    J = - w * JnormD * ( JposA - JposB );
  }

  // free
  delete m1;
  delete m2;
}

CollisionObject * ShapePairFCL::createObjectModel( rai::Shape * s )
{
  CHECK( s, "" );

  auto mesh = s->sscCore;

  CHECK( mesh.V.N, "" );

  /// 1 - set mesh triangles and vertice indices
  std::vector<Vec3f> vertices;
  std::vector<Triangle> triangles;

  if( vertices_.count( s ) == 0 )
  {
    // code to set the vertices and triangles
    for( uint i = 0; i < mesh.V.d0; i++ )
    {
      vertices.push_back( Vec3f( mesh.V( i, 0 ), mesh.V( i, 1 ), mesh.V( i, 2 ) ) );
    }

    for( uint i = 0; i < mesh.T.d0; i++ )
    {
      triangles.push_back( Triangle( mesh.T( i, 0 ), mesh.T( i, 1 ), mesh.T( i, 2 ) ) );
    }

    vertices_[ s ] = vertices;
    triangles_[ s ] = triangles;
  }
  else
  {
    vertices = vertices_[ s ];
    triangles = triangles_[ s ];
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
}*/
