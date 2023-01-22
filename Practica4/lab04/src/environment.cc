/*
 * environment.cc (C) Inaki Rano 2022
 * 
 * Definition of environment class for the 2D robot simulator
 * and the classes related to the environment (obstacles)
 *
 */
#include <cmath>
#include <iostream>
#include "environment.hh"

namespace rs {

  // Pointer to the environment initilised to null until an
  // environment object is created
  Environment *Environment::m_envp = NULL;

  // Method in the segment class to compute intersection with other
  // types of objects/obstacles
  bool
  Segment::intersection(const ObjectPtr & o, Position & p) const
  {
    switch (o->type())
      {
	// Intersection with another segment
      case RS_OBJECT_SEGMENT:
	{
	  SegmentPtr S = std::dynamic_pointer_cast<Segment>(o);
	  Position dp0(S->m_p0 - m_p0);
	  Eigen::Matrix2f A;
	  A << m_p1[0] - m_p0[0],
	    S->m_p0[0] - S->m_p1[0],
	    m_p1[1] - m_p0[1],
	  S->m_p0[1] - S->m_p1[1];
	  //std::cout << "p0 " << m_p0.transpose() << std::endl;
	  //std::cout << "p1 " << m_p1.transpose() << std::endl;
	  //std::cout << "Sp0 " << S->m_p0.transpose() << std::endl;
	  //std::cout << "Sp1 " << S->m_p1.transpose() << std::endl;
	  //std::cout << "A " << A << std::endl;
	  Position sol = A.colPivHouseholderQr().solve(dp0);
	  //std::cout << "Sol: " << sol.transpose() << std::endl;

	  // This method fails if the matrix A is rank deficient. From
	  // the simulations and tests this can be solved with a hack,
	  // setting the conditions 'sol[] > 0' instead of 'sol[] >=
	  // 0'. This means an intersection at the beginning of the
	  // segment is not possible (and the method isIn() for the
	  // polygons was changed accordingly)
	  if ((sol[0] > 0) && (sol[0] <= 1) &&
	      (sol[1] > 0) && (sol[1] <= 1))
	    {
	      p = m_p0 + sol[0] * (m_p1 - m_p0);
	      return true;
	    }
	}
	break;
	// Intersection with a circle
      case RS_OBJECT_CIRCLE:
	{
	CirclePtr C = std::dynamic_pointer_cast<Circle>(o);
	// All these formulas are derived and carefully reported in
	// some scrap paper in my office.
	Position p1p0(m_p1 - m_p0);
	Position p0c(m_p0 - C->m_c);
	float a(p1p0.norm());
	a *= a;
	float b(2*p1p0.dot(p0c));
	float c(p0c.norm());
	c *= c;
	c -= C->m_r * C->m_r;
	float disc(b * b - 4 * a *c);
	// disc<0 means complex solutions, i.e. no intersection
	if (disc >= 0)
	  {
	    float l0((-b + sqrt(disc)) / (2*a)); // Solution 1
	    float l1((-b - sqrt(disc)) / (2*a)); // Solution 2
	    // Intersects in two points, we've to take the closest to p0
	    if (((l0 >= 0) && (l0 <= 1)) &&
		((l1 >= 0) && (l1 <= 1)))
	      {
		float lmin((l0<l1) ? l0 : l1);
		p = m_p0 + lmin * (m_p1 - m_p0);
		return true;
	      }
	    // No intersection with the segment (still, it intersects
	    // with the infinite line, but not interested)
	    if (((l0 < 0) || (l0 > 1)) &&
		((l1 < 0) || (l1 > 1)))
	      return false;
	    // Intersects in one point
	    if ((l0 >= 0) && (l0 <= 1))
	      p = m_p0 + l0 * (m_p1 - m_p0);
	    else
	      p = m_p0 + l1 * (m_p1 - m_p0);	    
	    
	    return true;
	  }
	}
	break;
	// Intersection with a polygon
      case RS_OBJECT_POLYGON:
	{
	  PolygonPtr P = std::dynamic_pointer_cast<Polygon>(o);
	  bool retVal(false);
	  Position pInt(m_p1); // set the intersection at the end of
			       // the segmene
	  double dist((m_p0 - m_p1).norm()); // intersection distance
					     // initialy length of the
					     // segment
	  int sz(P->m_pts.size());
	  // Build segments with consecutive pairs of points and
	  // compute intersection with each one
	  for (int ii = 1; ii < sz; ii++)
	    {
	      SegmentPtr s1 = std::make_shared<Segment>(P->m_pts[ii-1],
							P->m_pts[ii]);
	      Position pIntNew;
	      if (this->intersection(s1, pIntNew))
		{
		  // If the new intersection point is closer to p0
		  // than the previous one, update the intersection
		  // point and the distance
		  if ((m_p0 - pIntNew).norm() < dist)
		    {
		      retVal = true;
		      dist = (m_p0 - pIntNew).norm();
		      pInt = pIntNew;
		    }
		}
	    }
	  // Build a segment from the first point to the last,
	  // i.e. closed polygon, to check for intersection. To allow
	  // open polygons this part below should be conditional
	  SegmentPtr s1 = std::make_shared<Segment>(P->m_pts[0],
						    P->m_pts[sz - 1]);
	  Position pIntNew;
	  if (intersection(s1, pIntNew))
	    {
	      if ((m_p0 - pIntNew).norm() < dist)
		{
		  retVal = true;
		  dist = (m_p0 - pIntNew).norm();
		  pInt = pIntNew;
		}
	    }
	  if (retVal)
	    {
	      p = pInt;
	      return true;
	    }
	}
	break;
      default:
	std::cout << "Segment::intersetion(); Not implemented." << std::endl;
    }
    
    return false;
  }

  bool
  Segment::isIn(const Position & p, float r) const
  {
    Position u(m_p1 - m_p0);
    u /= u.norm();
    Position dp(p - m_p0);
    //float dist(u.dot(dp));
    float aux(u[0]);
    u[0] = -u[1];
    u[1] = aux;
    float distPer(u.dot(dp));
    if ((fabs(distPer) < r)) // Other conditions are missing
      return true;
    return false;
    
  }

  bool
  Circle::isIn(const Position & p, float r) const
  {
    return (p - m_c).norm() < (r + m_r);
  }

  bool
  Polygon::isIn(const Position & p, float r) const
  {
    float xMin(p[0]), xMax(p[0]);
    for (int ii = 0; ii < m_pts.size(); ii++)
      {
	double foo(m_pts[ii][0]);
	if (foo < xMin)
	  xMin = foo;
	if (foo > xMax)
	  xMax = foo;
      }
    if ((xMin == p[0]) || (xMax == p[0]))
      return false;
    
    Segment s0(Position(xMin-0.1, p[1]), Position(xMax+0.1, p[1]));
    int count(0);
    int sz(m_pts.size());
    Position pt;
    for (int ii = 1; ii < sz; ii++)
      {
	SegmentPtr s1 = std::make_shared<Segment>(m_pts[ii-1], m_pts[ii]);
	if ((s0.intersection(s1, pt)) && (pt[0] < p[0]))
	  count++;
      }
    SegmentPtr s1 = std::make_shared<Segment>(m_pts[0], m_pts[sz-1]);
    if ((s0.intersection(s1, pt)) && (pt[0] < p[0]))
      count++;
    if (count & 0x1)
      return true;
    
    return false;
  }
  
  // Constructor sets the limit points, size and pointer to the
  // environment. Checks validity of the arguments and that there are
  // no more than one environment object
  Environment::Environment(const Position & min, const Position & max)
  {
    assert(m_envp == NULL);
    for (unsigned int ii = 0; ii < min.size(); ii++)
      assert(min[ii] < max[ii]);
    m_min = min;
    m_max = max;
    m_size = max - min;
    m_envp = this;
  }

  
  
  bool
  Environment::isFree(const Position & x, double r) const
  {
    for (int ii = 0; ii < this->size(); ii++)
      if ((*this)[ii]->isIn(x, r))
	return false;

    return true;
  }

  Position
  Environment::random() const
  {
    return Position(0.5 * (m_min + m_max) +
		      Position::Random().cwiseProduct(0.5 * m_size));
  }

  Position
  Environment::random(float r, int & trials) const
  {
    Position p;
    bool isPFree(true);
    do {
      p = random();
      isPFree = isFree(p, r);
    } while ((trials-- > 0) &&(!isPFree));

    return p;
  }
  
  
  std::ostream & operator<< (std::ostream & ofd, const rs::Environment & env)
  {
    ofd << "Environment: " << "minP: " << env.min().transpose() << std::endl;
    ofd << "Environment: " << "maxP: " << env.max().transpose() << std::endl;
	 
    return ofd;
  }
  

}
