/*
 * DAPlayground.h
 *
 *  Created on: 25 oct. 2014
 *      Author: ludo6431
 */

#ifndef DAPLAYGROUND_H_
#define DAPLAYGROUND_H_

#include <cairomm/context.h>
#include <cairomm/refptr.h>
#include <gdk/gdk.h>
#include <gtkmm/drawingarea.h>

class DAPlayground: public Gtk::DrawingArea {

public:
	DAPlayground();
	virtual ~DAPlayground();

    // ### public methods ###
	// rendering management
	void invalidate_all();

	// point selection
	void event_click__px(double x, double y);
	bool state_click_moved();
	void state_get_click__cm(double& x, double& y);

	// hover
	void event_hover__px(double x, double y);
	bool state_hover_moved();
	void state_get_hover__cm(double& x, double& y);
	void state_get_hover__cm(float& x, float& y);

	// panning
	bool state_is_panning() const;
	void event_pan_start__px(double x, double y);
	void event_pan_update_abs__px(double x, double y);
	void event_pan_stop();

	// scaling
	void event_scale_rel(double s);

	// accessors
	double world_width__cm() const{
	    return wld_width__cm;
	}
    double world_height__cm() const{
        return wld_height__cm;
    }

protected:
	// ### signal handlers ###
	bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr);
	bool on_button_press_event(GdkEventButton* event);
	bool on_motion_notify_event(GdkEventMotion* event);
	bool on_button_release_event(GdkEventButton* event);
	bool on_scroll_event(GdkEventScroll* event);

	// ### protected methods ###
	void prepare_draw(const Cairo::RefPtr<Cairo::Context>& cr);

	// ### protected attributes ###
    int widget_width__px, widget_height__px; // (px)

    // world
    double wld_width__cm, wld_height__cm; // (cm)
    double scale; // (px/cm)
    double center_x__cm, center_y__cm; // (cm)
    bool center_px_moved;
    double start_x__px, start_y__px; // (px)
    double center_x_incr__px, center_y_incr__px; // (px)
    bool mouse_bt1_dragging;
    bool center_cm_moved;
    double center_x_incr__cm, center_y_incr__cm; // (cm)

    // mouse interaction
    bool mouse_lastpress_moved, user_mouse_lastpress_moved;
    double mouse_lastpress_x__px, mouse_lastpress_y__px; // (px)
    double mouse_lastpress_x__cm, mouse_lastpress_y__cm; // (cm)
    bool mouse_moved, user_mouse_moved;
    double mouse_x__px, mouse_y__px; // (px)
    double mouse_x__cm, mouse_y__cm; // (cm)
};

#endif /* DAPLAYGROUND_H_ */
