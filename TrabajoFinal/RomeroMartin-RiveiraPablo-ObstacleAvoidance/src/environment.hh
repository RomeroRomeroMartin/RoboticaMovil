/*
 * environment.hh (C) Inaki Rano 2022
 * 
 * Declaration of environment class for the 2D robot simulator
 * 
 * The environment is where the robot will be simulated and the
 * obstacles that can be placed in it
 *
 */ 
#ifndef RS_ENVIRONMENT_HH
#define RS_ENVIRONMENT_HH
#include <vector>
#include <memory>
#include "base.hh"

// Types of obstacles in the environment (Segments are not used as
// obtacles but used by polygons)
#define RS_OBJECT_SEGMENT 0
#define RS_OBJECT_CIRCLE  1
#define RS_OBJECT_POLYGON 2


namespace rs {

  // Abstract class for objects in the environment
  class Object {
  public:
    // Constructor stores the type of obstacle
    Object(int type) {m_type = type;}

    // Desctructor for virtual class
    virtual ~Object() {};

    // Returns the type of obstacle
    int type() const {return m_type;}

    // Checks if the position given is inside the obstacle (radius is
    // somethimes ignored)
    virtual bool isIn(const Position &, float r) const = 0;
    
  private:
    int m_type; // Type of obstacle
  };
  // Shared pointer, can be defined inside the class itself?
  typedef std::shared_ptr<Object> ObjectPtr;

  // Segment class used for polygons and to simulate a laser scanner,
  // never tried to include a segment obstacle in the environment but
  // it should work (except for the GUI)
  class Segment : public Object {
  public:
    // Constructor initial point of the segment (p0) and end point
    // (p1)
    Segment(const Position & p0, const Position & p1) :
      Object(RS_OBJECT_SEGMENT)
    {
      m_p0 = p0;
      m_p1 = p1;
    }
    // Destructor, really?
    ~Segment() {};

    // Compute the intersection of the segment with different objects
    // (see below). Returns true if the segment intersects with the
    // object and the position argument is set to the intersection
    // point closer to the initial point of the segment (p0)
    bool intersection(const ObjectPtr &, Position &) const;

    // Check if the point given as position belongs to the segment
    // (not fully working)
    bool isIn(const Position &, float r) const;
    
  private:
    Position m_p0; // Begining of the segment
    Position m_p1; // End of the segment
  };
  // Pointer to the segment
  typedef std::shared_ptr<Segment> SegmentPtr;

  // Circle obstacle.
  class Circle : public Object {
  public:
    // Constructor of the circle using center and radius
    Circle(const Position & c, float r) :
      Object(RS_OBJECT_CIRCLE)
    { m_c = c; m_r = r; }
    ~Circle() {};
    
    // Checks if the point given is inside the circle
    bool isIn(const Position &, float r) const;

    Position m_c;
    float m_r;
  };
  // Smart pointer to the circle class
  typedef std::shared_ptr<Circle> CirclePtr;

  // Polygon class, stored as a list of points. Polygons in this
  // implementation are closed (i.e. last point connected to first),
  // but this can be changed to have open polygons
  class Polygon: public Object {
  public:
    // Constructor requires a standard vector of points (positions)
    Polygon(const std::vector<Position> & pts) :
      Object(RS_OBJECT_POLYGON)
    {
      for (int ii = 0; ii < pts.size(); ii++)
	m_pts.push_back(pts[ii]);
    }
    
    // Check if the point (position) given is inside the polygon (made
    // by counting the number of intersections with an horizontal line
    // going through the point)
    bool isIn(const Position &, float) const;

    std::vector<Position> m_pts;
  };
  // Smart pointer to a polygon
  typedef std::shared_ptr<Polygon> PolygonPtr;
    
  // Declaration of the class environment, derived from a standard
  // vector of objects/obstacles
  class Environment : public std::vector<ObjectPtr> {
  public:
    // Constructor
    Environment(const Position & min, const Position & max);
    
    // Methods to get the limits of the environment.
    // Minimum bottom-left-most point
    const Position & min() const {return m_min;}
    // Maximum top-right-most point
    const Position & max() const {return m_max;}

    // Method to check a circular area in the environment is free
    // The position is the centre of the circular area with radius "r"
    bool isFree(const Position &, double r = 0) const;

    // Generate a random position in the environment
    Position random() const;
    
    // Generate a random position in the environment. The position
    // will be the centre of a free area of radius "r" given the
    // environment. The method will try to generate points "trials"
    // times. If on return trials <=0 the position generated is
    // invalid, i.e. no free position was randomly found.
    Position random(float r, int & trials) const;

  protected:

    friend std::ostream & operator<< (std::ostream & , const rs::Environment & );


    // Singleton pattern, there must be only one environment to simulate.
    static Environment *m_envp;

  private:
    // Default constructor and copy constructors are deleted to
    // implement the singleton pattern (seems to be a cheap way)
    Environment() = delete;
    Environment(const Environment & e) = delete;
    
    Position m_min, m_max;
    Position m_size;
  };

  // Overloaded operator to write the environment into a stream
  std::ostream & operator<< (std::ostream & , const rs::Environment & );

  typedef std::shared_ptr<Environment> EnvironmentPtr;

}

#endif
