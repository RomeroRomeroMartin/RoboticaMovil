/*
 * simulation-plot.hh (C) Inaki Rano 2022
 * 
 * Definition of the classes to display a simulation.
 *
 */
#include <stdlib.h>
#include <sstream>
#include <iomanip>

#include <iostream>    

#include "environment.hh"
#include "simulation-plot.hh"

namespace rs {

  SimulationDraw::SimulationDraw(const SimulationPtr &sim,
				 const Position &min,
				 const Position & max)
  {
    //m_cairoContext = get_window()->create_cairo_context();
    m_simulationPtr = sim;
    m_min = min;
    m_max = max;
    m_size = Position(600, 600);
    m_t = 0.0;
    m_speed = 1;
  }

  SimulationDraw::~SimulationDraw()
  {}

  void
  SimulationDraw::draw(float dt, float speed)
  {
    m_t += dt;
    m_speed = speed;
  }
  
  bool
  SimulationDraw::on_draw(const Cairo::RefPtr<Cairo::Context> & cr)
  {    
    Gtk::Allocation allocation = get_allocation();
    m_size[0] = (float)allocation.get_width();
    m_size[1] = (float)allocation.get_height();
    
    Position scale(m_size.cwiseQuotient(m_max - m_min));
    //Position c(m_size/2);
    Pose p((*m_simulationPtr)[m_t]);
    std::vector<float> scan(m_simulationPtr->scan(m_t));

    cr->save();
    Pango::FontDescription font;

    font.set_family("Monospace");
    font.set_weight(Pango::WEIGHT_BOLD);
    std::stringstream stream;
    stream << "Play Speed: "
      << std::fixed << std::setprecision(2) << m_speed
      << " Time: " << m_t;

    auto layout = create_pango_layout(stream.str().c_str());
    layout->set_font_description(font);
    cr->move_to(0,0);
    layout->show_in_cairo_context(cr);
    cr->stroke();
    
    for (unsigned int ii = 0; ii < m_simulationPtr->environment()->size(); ii++)
      {
	ObjectPtr op = m_simulationPtr->environment()->operator[](ii);
	switch (op->type()) {
	case RS_OBJECT_CIRCLE:
	  {
	    CirclePtr C = std::dynamic_pointer_cast<Circle>(op);
	    Position rp(C->m_c);
	    double r(scale.minCoeff() * C->m_r);
	    Position flip(1, -1);
	    Position rc(m_size/2 +
			scale.cwiseProduct(flip.cwiseProduct(rp)));
	    cr->set_source_rgba(0, 0, 0, 1.0);
	    cr->arc(rc[0], rc[1], r, 0.0, 2.0 * M_PI); // full circle
	    cr->fill_preserve();
	    cr->stroke();
	    break;
	  }
	case RS_OBJECT_POLYGON:
	  {
	    PolygonPtr P = std::dynamic_pointer_cast<Polygon>(op);
	    cr->set_source_rgba(0.0, 0, 0, 1.0);
	    int sz(P->m_pts.size());
	    for (int jj = 1; jj < sz; jj++)
	      {
		Position flip(1, -1);
		Position rc0(m_size/2 +
			     scale.cwiseProduct(flip.cwiseProduct(P->m_pts[jj-1])));
		Position rc1(m_size/2 +
			     scale.cwiseProduct(flip.cwiseProduct(P->m_pts[jj])));
		cr->move_to(rc0[0], rc0[1]);
		cr->line_to(rc1[0], rc1[1]);
	      }
	    Position flip(1, -1);
	    Position rc0(m_size/2 +
			 scale.cwiseProduct(flip.cwiseProduct(P->m_pts[sz-1])));
	    Position rc1(m_size/2 +
			 scale.cwiseProduct(flip.cwiseProduct(P->m_pts[0])));
	    cr->move_to(rc0[0], rc0[1]);
	    cr->line_to(rc1[0], rc1[1]);
	    cr->stroke();
	    cr->fill();
	  }
	  break;
	default:
	  break;
	}
      }
    Eigen::VectorXf ang1 = Eigen::VectorXf::LinSpaced(scan.size(),
						      -M_PI, M_PI);
    unsigned inc(scan.size() / 90);
    for (unsigned int ii = 0; ii < scan.size(); ii += inc)
      {
	float ang0(p[2]);
	Position rp(p[0],p[1]);
	Position flip(1, -1);
	Position rc0(m_size/2 +
		     scale.cwiseProduct(flip.cwiseProduct(rp)));
	Position rp1(rp + Position(scan[ii] * cos(ang0 + ang1[ii]),
				   scan[ii] * sin(ang0 + ang1[ii])));
	Position rc1(m_size/2 +
		     scale.cwiseProduct(flip.cwiseProduct(rp1)));
	cr->set_source_rgba(1.0, 0, 0, 1.0);
	cr->move_to(rc0[0], rc0[1]);
	cr->line_to(rc1[0], rc1[1]);
	cr->stroke();
      }
    cr->restore();  // back to opaque black

    return true;
  }
  
  
  SimulationWindow::SimulationWindow(const SimulationPtr & sim):
    m_simulationDraw(sim, sim->environment()->min(), sim->environment()->max())
  {
    set_title("Robot Simulator");
    set_default_size(600, 600);
    set_resizable(true);
    signal_key_press_event().connect(sigc::mem_fun(*this, 
						   &SimulationWindow::key_pressed), false);
    add(m_simulationDraw);
    m_simulationDraw.show();
    m_speed = 0;
    m_base_ms = 1000 * sim->dt();
    m_maxSpeed = 9;
    m_playStatus = false;
    m_update_ms = 0;
    int update_ms(1000);
    Glib::signal_timeout().connect_once(sigc::mem_fun(*this,
						      &SimulationWindow::timeout),
					update_ms);
  }

  bool
  SimulationWindow::key_pressed(GdkEventKey * k)
  {
    switch (k->keyval) {
    case GDK_KEY_Up:
      m_speed++;
      m_speed = (m_speed > m_maxSpeed) ? m_maxSpeed : m_speed;
      break;
    case GDK_KEY_Right:
      m_speed++;
      m_speed = (m_speed > m_maxSpeed) ? m_maxSpeed : m_speed;
      break;
    case GDK_KEY_Down:
      m_speed--;
      m_speed = (m_speed < -m_maxSpeed) ? -m_maxSpeed : m_speed;
      break;
    case GDK_KEY_Left:
      m_speed--;
      m_speed = (m_speed < -m_maxSpeed) ? -m_maxSpeed : m_speed;
      break;
    case GDK_KEY_space:
      m_playStatus = !m_playStatus;
      break;
    case GDK_KEY_Escape:
      exit(0);
      break;
    }

    return true;
  }

  float
  SimulationWindow::computeSpeed()
  {
    if (m_speed == 0)
      return 1.0;
    if (m_speed > 0)
      return ((float)(1 + m_speed));
    
    return 1/((float)(1 - m_speed));
  }


  void
  SimulationWindow::timeout()
  {

    if (m_playStatus)
      {
	m_simulationDraw.draw(float(m_base_ms) / 1000, computeSpeed());
	get_window()->invalidate(true);
      }
    Glib::signal_timeout().connect_once(sigc::mem_fun(*this,
						      &SimulationWindow::timeout),
					m_update_ms);

    m_update_ms = int(m_base_ms / computeSpeed());    
  }
  
}
