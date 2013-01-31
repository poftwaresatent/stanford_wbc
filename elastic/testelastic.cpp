/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Roland Philippsen */

#include "elastic.hpp"
#include <opspace/Skill.hpp>
#include <gtk/gtk.h>
#include <cmath>
#include <iostream>
#include <list>
#include <err.h>

using namespace elastic;


static double const deg(M_PI / 180.);


class WaypointSkill;

class Waypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Waypoint(Model & model,
	   WaypointSkill & skill)
    : model_(model),
      skill_(skill),
      pos_a_(Vector::Zero(2)),
      pos_b_(Vector::Zero(2))
  {
  }
  
  void draw (cairo_t * cr, double pixelsize)
  {
    cairo_save (cr);
    
    cairo_set_source_rgba (cr, 0.7, 0.7, 0.7, 0.5);
    cairo_arc (cr, state_[0], state_[1], radius_, 0., 2. * M_PI);
    cairo_fill (cr);
    
    cairo_set_source_rgb (cr, 0.2, 0.2, 0.2);
    cairo_set_line_width (cr, 3.0 / pixelsize);
    cairo_arc (cr, state_[0], state_[1], radius_, 0., 2. * M_PI);
    cairo_stroke (cr);
    
    cairo_move_to (cr, state_[0], state_[1]);
    cairo_line_to (cr, pos_a_[0], pos_a_[1]);
    cairo_line_to (cr, pos_b_[0], pos_b_[1]);
    cairo_stroke (cr);
    
    cairo_set_source_rgb (cr, 1.0, 0.4, 0.4);
    cairo_set_line_width (cr, 1.0 / pixelsize);
    cairo_move_to (cr, task_[0].current[0], task_[0].current[1]);
    cairo_line_to (cr, task_[0].desired[0], task_[0].desired[1]);
    cairo_stroke (cr);
    
    cairo_set_source_rgb (cr, 0.4, 1.0, 0.4);
    cairo_move_to (cr, task_[1].current[0], task_[1].current[1]);
    cairo_line_to (cr, task_[1].desired[0], task_[1].desired[1]);
    cairo_stroke (cr);
    
    cairo_restore (cr);
  }
  
  
  void setState (Vector const & state)
  {
    if (state.size() != 4) {
      errx (EXIT_FAILURE, "only NDOF=4 allowed for now...");
    }
    state_ = state;
    
    double const ac2(len_a_ * cos(state_[2]));
    double const as2(len_a_ * sin(state_[2]));
    double const bc23(len_b_ * cos(state_[2] + state_[3]));
    double const bs23(len_b_ * sin(state_[2] + state_[3]));
    
    pos_a_ << state_[0] + ac2,  state_[1] + as2;
    pos_b_ << pos_a_[0] + bc23, pos_a_[1] + bs23;
    
    
    task_[0].current = pos_b_;
    task_[0].Jacobian <<
      1, 0, -as2 - bs23, -bs23,
      0, 1,  ac2 + bc23,  bc23;
    
    task_[1].current << state_[0], state_[1];
    
    ////    task_[2].current = state_;
  }
  
  
  //private:
  Vector state_;
  Vector pos_a_;
  Vector pos_b_;
};


class WaypointSkill
  : public GenericSkill
{
public:
  explicit WaypointSkill(std::string const & name)
    : GenericSkill(name),
      radius_(0.5),
      len_a_(0.8),
      len_b_(0.6)
  {
    declareParameter("radius", &radius_);
    declareParameter("len_a", &len_a_);
    declareParameter("len_b", &len_b_);
  }
  
  Status GenericSkill::
  init(Model const & model)
  {
    Status const st(GenericSkill::init(model));
    if ( ! st) {
      return st;
    }
    eepos_ = lookupTask<CartPosTask>("eepos");
    if ( ! eepos_) {
      return Status(false, "no matching eepos task (could be empty, wrong name, or wrong type)");
    }
    basepos_ = lookupTask<CartPosTask>("basepos");
    if ( ! basepos_) {
      return Status(false, "no matching basepos task (could be empty, wrong name, or wrong type)");
    }
    jpos_ = lookupTask<CartPosTask>("jpos");
    if ( ! jpos_) {
      return Status(false, "no matching jpos task (could be empty, wrong name, or wrong type)");
    }
    return st;
  }
  
  //protected:  or private:
  double radius_;
  double len_a_;
  double len_b_;
  
  CartPosTask * eepos_;
  CartPosTask * basepos_;
  JPosTask * jpos_;
};


// could templatize on Waypoint or pass in a factory or something along
// those lines to make it robot agnostic...
class Elastic
{
public:
  typedef list<Waypoint *> path_t;
  
  ~Elastic ()
  {
    clear();
  }
  
  void clear ()
  {
    for (path_t::iterator ii(path_.begin()); ii != path_.end(); ++ii) {
      delete *ii;
    }
    path_.clear();
  }
  
  void init (Vector const & ee0, Vector const & ee1,
	     Vector const & base0, Vector const & base1,
	     Vector const & posture,
	     size_t nsteps)
  {
    if (0 == nsteps) {
      nsteps = 1;
    }
    clear();
    
    start_ = new Waypoint(posture);
    dest_ = new Waypoint(posture);
    
    Vector delta_ee = (ee1 - ee0) / nsteps;
    Vector delta_base = (base1 - base0) / nsteps;
    
    Vector ee = ee0 + delta_ee;
    Vector base = base0 + delta_base;
    
    path_.push_back (start_);
    for (size_t ii(1); ii < nsteps; ++ii) {
      Waypoint * wpt(new Waypoint(posture));
      wpt->setEEGoal(ee);
      wpt->setBaseGoal(base);
      path_.push_back (wpt);
      ee += delta_ee;
      base += delta_base;
    }
    path_.push_back (dest_);
  }
  
  void update (Vector const & ee0, Vector const & ee1,
	       Vector const & base0, Vector const & base1)
  {
    start_->setEEGoal (ee0);
    start_->setBaseGoal (base0);
    dest_->setEEGoal (ee1);
    dest_->setBaseGoal (base1);
    cout << "==================================================\n";
    for (path_t::iterator ii(path_.begin()); ii != path_.end(); ++ii) {
      if (start_ == *ii) {
	cout << "START\n";
      }
      else if (dest_ == *ii) {
	cout << "DESTINATION\n";
      }
      else {
	cout << "waypoint\n";
      }
      Vector dq = Vector::Ones((*ii)->getState().size()) * 0.02;
      (*ii)->setState ((*ii)->getState() + dq);
      cout << "--------------------------------------------------\n";
    }
  }
  
  void draw (cairo_t * cr, double pixelsize)
  {
    for (path_t::reverse_iterator ii(path_.rbegin()); ii != path_.rend(); ++ii) {
      (*ii)->draw(cr, pixelsize);
    }
  }
  
private:
  path_t path_;
  Waypoint * start_;
  Waypoint * dest_;
};


static double const dimx(10.);
static double const dimy(8.);

static GtkWidget * gw(0);
static gint gw_width(800), gw_height(640);
static gint gw_sx, gw_sy, gw_x0, gw_y0;
static int play(0);
static Vector start_ee(2);
static Vector dest_ee(2);
static Vector start_base(2);
static Vector dest_base(2);
static Vector posture(4);
static Vector * handle[] = { &start_ee, &dest_ee, &start_base, &dest_base, 0 };
static Vector * grabbed(0);
static double grab_radius(0.2);
static Vector grab_offset(2);


static Elastic elastic;


static void update ()
{
  elastic.update (start_ee, dest_ee, start_base, dest_base);
  gtk_widget_queue_draw (gw);
}


static void cb_play (GtkWidget * ww, gpointer data)
{
  if (play) {
    play = 0;
  }
  else {
    play = 1;
  }
}


static void cb_next (GtkWidget * ww, gpointer data)
{
  if (play) {
    play = 0;
  }
  else {
    update ();
  }    
}


static void cb_quit (GtkWidget * ww, gpointer data)
{
  gtk_main_quit();
}


static gint cb_expose (GtkWidget * ww,
		       GdkEventExpose * ee,
		       gpointer data)
{
  cairo_t * cr = gdk_cairo_create (ee->window);
  
  cairo_set_source_rgb (cr, 1.0, 1.0, 1.0);
  cairo_rectangle (cr, 0, 0, gw_width, gw_height);
  cairo_fill (cr);
  
  cairo_translate (cr, gw_x0, gw_y0);
  cairo_scale (cr, gw_sx, gw_sy);
  
  cairo_set_source_rgb (cr, 0.6, 0.0, 0.0);
  cairo_set_line_width (cr, 1.0 / gw_sx);
  cairo_move_to (cr, start_ee[0], start_ee[1]);
  cairo_line_to (cr, dest_ee[0], dest_ee[1]);
  cairo_stroke (cr);
  
  cairo_set_source_rgb (cr, 0.0, 0.6, 0.0);
  cairo_move_to (cr, start_base[0], start_base[1]);
  cairo_line_to (cr, dest_base[0], dest_base[1]);
  cairo_stroke (cr);
  
  elastic.draw(cr, gw_sx);
  
  cairo_set_source_rgba (cr, 0.6, 0.0, 0.0, 0.5);
  cairo_arc (cr, start_ee[0], start_ee[1], grab_radius, 0., 2. * M_PI);
  cairo_fill (cr);
  cairo_arc (cr, dest_ee[0], dest_ee[1], grab_radius, 0., 2. * M_PI);
  cairo_fill (cr);
  
  cairo_set_source_rgba (cr, 0.0, 0.6, 0.0, 0.5);
  cairo_arc (cr, start_base[0], start_base[1], grab_radius, 0., 2. * M_PI);
  cairo_fill (cr);
  cairo_arc (cr, dest_base[0], dest_base[1], grab_radius, 0., 2. * M_PI);
  cairo_fill (cr);
  
  cairo_destroy (cr);
  
  return TRUE;
}


static gint cb_size_allocate (GtkWidget * ww,
			      GtkAllocation * aa,
			      gpointer data)
{
  gw_width = aa->width;
  gw_height = aa->height;
  
  gw_sx = gw_width / dimx;
  if (gw_sx < 1) {
    gw_sx = 1;
  }
  gw_sy = - gw_height / dimy;
  if ( - gw_sy < 1) {
    gw_sy = -1;
  }
  if (gw_sx > - gw_sy) {
    gw_sx = - gw_sy;
  }
  else {
    gw_sy = - gw_sx;
  }
  gw_x0 = (gw_width - dimx * gw_sx) / 2;
  gw_y0 = gw_height - (gw_height + dimy * gw_sy) / 2;
  
  return TRUE;
}


static gint cb_click (GtkWidget * ww,
		      GdkEventButton * bb,
		      gpointer data)
{
  if (bb->type == GDK_BUTTON_PRESS) {
    Vector point(2);
    point << (bb->x - gw_x0) / (double) gw_sx, (bb->y - gw_y0) / (double) gw_sy;
    for (Vector ** hh(handle); *hh != 0; ++hh) {
      Vector offset = **hh - point;
      if (offset.norm() <= grab_radius) {
    	grab_offset = offset;
    	grabbed = *hh;
    	break;
      }
    }
  }
  else if (bb->type == GDK_BUTTON_RELEASE) {
    grabbed = 0;
    cout << "Why don't I get GDK_BUTTON_RELEASE (under OSX)?\n";
  }
  
  return TRUE;
}


static gint cb_motion (GtkWidget * ww,
		       GdkEventMotion * ee)
{
  int mx, my;
  GdkModifierType modifier;
  gdk_window_get_pointer (ww->window, &mx, &my, &modifier);
  
  if (0 != grabbed) {
    Vector point(2);
    point << (mx - gw_x0) / (double) gw_sx, (my - gw_y0) / (double) gw_sy;
    *grabbed = point + grab_offset;
    gtk_widget_queue_draw (gw);
  }
  
  return TRUE;
}


static gint idle (gpointer data)
{
  if (play) {
    update ();
  }
  return TRUE;
}


static void init_gui (int * argc, char *** argv)
{
  GtkWidget *window, *vbox, *hbox, *btn;
  
  gtk_init (argc, argv);
  
  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  
  vbox = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox);
  gtk_widget_show (vbox);
  
  gw = gtk_drawing_area_new ();
  g_signal_connect (gw, "expose_event", G_CALLBACK (cb_expose), NULL);
  g_signal_connect (gw, "size_allocate", G_CALLBACK (cb_size_allocate), NULL);
  g_signal_connect (gw, "button_press_event", G_CALLBACK (cb_click), NULL);
  g_signal_connect (gw, "motion_notify_event", G_CALLBACK (cb_motion), NULL);
  gtk_widget_set_events (gw,
			 GDK_BUTTON_PRESS_MASK |
			 GDK_BUTTON_RELEASE_MASK |
			 GDK_BUTTON_MOTION_MASK);
  
  gtk_widget_show (gw);
  
  gtk_widget_set_size_request (gw, gw_width, gw_height);
  gtk_box_pack_start (GTK_BOX (vbox), gw, TRUE, TRUE, 0);
  
  hbox = gtk_hbox_new (TRUE, 3);
  gtk_box_pack_start (GTK_BOX (vbox), hbox, FALSE, TRUE, 0);
  gtk_widget_show (hbox);
  
  // btn = gtk_button_new_with_label ("reset");
  // g_signal_connect (btn, "clicked", G_CALLBACK (cb_reset), NULL);
  // gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  // gtk_widget_show (btn);
  
  btn = gtk_button_new_with_label ("play");
  g_signal_connect (btn, "clicked", G_CALLBACK (cb_play), NULL);
  gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  gtk_widget_show (btn);
  
  btn = gtk_button_new_with_label ("next");
  g_signal_connect (btn, "clicked", G_CALLBACK (cb_next), NULL);
  gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  gtk_widget_show (btn);
  
  btn = gtk_button_new_with_label ("quit");
  g_signal_connect (btn, "clicked", G_CALLBACK (cb_quit), NULL);
  gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  gtk_widget_show (btn);
  
  gtk_idle_add (idle, 0);
  
  gtk_widget_show (window);
}


int main (int argc, char ** argv)
{
  init_gui (&argc, &argv);
  
  start_ee   << 1.0,        dimy - 1.0;
  dest_ee    << dimx - 1.0, 2.0;
  start_base << 1.0,        dimy - 2.0;
  dest_base  << dimx - 1.0, 1.0;
  posture    << dimx / 2.0, dimy / 2.0, 80.0 * deg, - 40.0 * deg;
  
  elastic.init (start_ee, dest_ee, start_base, dest_base, posture, 1);//7);
  
  gtk_main ();
  
  return 0;
}
