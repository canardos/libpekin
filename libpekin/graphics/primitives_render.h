/**
 * @file
 *
 * Caution
 * -------
 * This code was developed for a specific application and has only been tested
 * to the the extent of that application's functionality. In other words it is
 * largely untested, likely contains minor bugs and inefficiencies and should
 * be tested carefully in any application you choose to use it in.
 *
 */
#ifndef PAINTERCOLOR_H_
#define PAINTERCOLOR_H_

#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include "graphics.h"
#include "misc_math.h"
#include "idrawing_surface.h"

namespace Libp {

/**
 * Renders primitive shapes on an IDrawingSurface.
 *
 * Warning:
 * -------
 * This code uses C99 VLAs in various places. These are not valid C++, but are
 * supported by GCC, and some other compilers.
 *
 * The stack space requirements may be significant. See individual member
 * function comments for details.
 *
 * @tparam T type of color used by the IDrawingSurface
 */
template <typename T>
class PrimitivesRender {
public:
    PrimitivesRender(IDrawingSurface<T>& surface) : surface_(surface) { }

    /** Draw a one pixel circle */
    void drawCircle(uint16_t x, uint16_t y, uint16_t radius, T color)
    {
    	// Bresenham’s Algorithm
		int16_t d = 1 - radius;
		int16_t xpos = radius;
		int16_t ypos = 0;
		do {
			surface_.setPixel(x + xpos, y + ypos, color); //  90 -> 135
			surface_.setPixel(x + xpos, y - ypos, color); //  45 ->  90

			surface_.setPixel(x - xpos, y + ypos, color); // 225 -> 270
			surface_.setPixel(x - xpos, y - ypos, color); // 270 -> 315

			surface_.setPixel(x + ypos, y + xpos, color); // 135 -> 180
			surface_.setPixel(x + ypos, y - xpos, color); //   0 ->  45

			surface_.setPixel(x - ypos, y + xpos, color); // 180 -> 225
			surface_.setPixel(x - ypos, y - xpos, color); // 315 -> 360
			ypos++;
			d = (d < 0)
				? d + 2 * ypos + 1
				: d + 2 * (ypos - --xpos) + 1;
		}
		while (xpos >= ypos);
    }

    /// Warning: uses >4*radius bytes in stack space
    void drawCircleThick(uint16_t x, uint16_t y, uint16_t outer_radius, uint16_t thickness, T color)
    {
    	if (thickness == 0) {
    		return;
    	}
    	if (thickness == 1) {
    		drawCircle(x, y, outer_radius, color);
    		return;
    	}
    	if (thickness >= outer_radius) {
    		drawCircleSolid(x, y, outer_radius, color);
    		return;
    	}
		uint16_t inner_radius = outer_radius - thickness + 1;

		// TODO: modify algo to draw lines as we go

		// TODO: we need to limit the radius
	    // Warning: C99 VLA array is not valid C++
		// and risks stack corruption
	    // Confirm support if not using GCC
    	int16_t x_outer[outer_radius + 1];
    	int16_t x_inner[inner_radius + 1];

    	// Bresenham’s Algorithm

    	// --- Calc outer circle ---
		int16_t d = 1 - outer_radius;
		int16_t xpos = outer_radius;
		int16_t ypos = 0;
		do {
			x_outer[xpos] = ypos;
			x_outer[ypos++] = xpos;
			//ypos++;
			d = (d < 0)
				? d + 2 * ypos + 1
				: d + 2 * (ypos - --xpos) + 1;

		}
		while (xpos >= ypos);

    	// --- Calc inner circle ---
		d = 1 - inner_radius;
		xpos = inner_radius;
		ypos = 0;
		do {
			x_inner[ypos++] = xpos;
			//ypos_b++;
			if (d < 0) {
				d = d + 2 * ypos + 1;
			}
			else {
				d = d + 2 * (ypos - --xpos) + 1;
				x_inner[xpos] = ypos;
			}
		}
		while (xpos >= ypos);

		// --- Draw ring ---
		for (int16_t yoffs = outer_radius; yoffs >= 0; yoffs--) {
			if (yoffs < inner_radius) {
				uint16_t length = x_outer[yoffs] - x_inner[yoffs] + 1;
				drawLineHoriz(x - x_outer[yoffs], y + yoffs, length, color);
				drawLineHoriz(x + x_outer[yoffs] - length + 1, y + yoffs, length, color);
				drawLineHoriz(x - x_outer[yoffs], y - yoffs, length, color);
				drawLineHoriz(x + x_outer[yoffs] - length + 1, y - yoffs, length, color);
			}
			else {
				drawLineHoriz(x - x_outer[yoffs], y + yoffs, x_outer[yoffs] * 2 + 1, color);
				drawLineHoriz(x - x_outer[yoffs], y - yoffs, x_outer[yoffs] * 2 + 1, color);
			}
		}
    }

    void drawCircleSolid(uint16_t x, uint16_t y, uint16_t radius, T color)
    {
    	// Bresenham’s Algorithm
		int16_t d = 1 - radius;
		int16_t xpos = radius;
		int16_t ypos = 0;
		do {
			surface_.fillLine(x - xpos, y + ypos, xpos * 2 + 1, color);
			surface_.fillLine(x - xpos, y - ypos, xpos * 2 + 1, color);
			surface_.fillLine(x - ypos, y + xpos, ypos * 2 + 1, color);
			surface_.fillLine(x - ypos, y - xpos, ypos * 2 + 1, color);
			ypos++;
			d = (d < 0)
				? d + 2 * ypos + 1
				: d + 2 * (ypos - --xpos) + 1;
		}
		while (xpos >= ypos);
    }

    /*
     * Below function adapted from LittlevGL library:
     *
     * https://github.com/littlevgl/lvgl
     *
     * License
     * -------
     *
     * MIT licence
     * Copyright (c) 2016 Gábor Kiss-Vámosi
     *
     * Permission is hereby granted, free of charge, to any person obtaining a
     * copy of this software and associated documentation files (the
     * “Software”), to deal in the Software without restriction, including
     * without limitation the rights to use, copy, modify, merge, publish,
     * distribute, sublicense, and/or sell copies of the Software, and to
     * permit persons to whom the Software is furnished to do so, subject to
     * the following conditions:
     *
     * The above copyright notice and this permission notice shall be included
     * in all copies or substantial portions of the Software.
     *
     * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
     * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
     * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
     * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
     * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
     * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
     * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
     */

    /**
     * Draw an arc. (Can draw pie too with great thickness.)
     *
     * @param x the x coordinate of the center of the arc
     * @param y the y coordinate of the center of the arc
     * @param radius the radius of the arc
     * @param start_angle the start angle of the arc (0 deg on the bottom, 90 deg on the right)
     * @param end_angle the end angle of the arc
     */
    void drawArc(uint16_t x, uint16_t y, uint16_t radius, uint16_t start_angle, uint16_t end_angle, uint16_t thickness, T color)
    {
        static constexpr int16_t LV_COORD_MIN = 0;
        if (thickness > radius)
            thickness = radius;

        int16_t r_outer = radius;
        int16_t r_inner = r_outer - thickness;

        bool (*deg_test)(uint16_t, uint16_t, uint16_t);
        deg_test = (start_angle <= end_angle)
            ? deg_test_norm
            : deg_test_inv;

        int middle_r_out = r_outer;
        // LittlevGL H/V line functions' len param is actually len-1
        // thickness--;
        middle_r_out = r_outer - 1;
        if (deg_test(270, start_angle, end_angle))
            drawLineHoriz(x - middle_r_out, y, thickness, color); // Left middle
        if (deg_test(90, start_angle, end_angle))
            drawLineHoriz(x + r_inner, y, thickness, color);      // Right middle
        if (deg_test(180, start_angle, end_angle))
            drawLineVert(x, y - middle_r_out, thickness, color);  // Top middle
        if (deg_test(0, start_angle, end_angle))
            drawLineVert(x, y + r_inner, thickness, color);       // Bottom middle

        uint32_t r_out_sqr = r_outer * r_outer;
        uint32_t r_in_sqr = r_inner * r_inner;

        int16_t x_start[4];
        int16_t x_end[4];
        int16_t deg_base;
        int16_t deg;

        for (int16_t yi = -r_outer; yi < 0; yi++) {
            x_start[0] = LV_COORD_MIN;
            x_start[1] = LV_COORD_MIN;
            x_start[2] = LV_COORD_MIN;
            x_start[3] = LV_COORD_MIN;
            x_end[0] = LV_COORD_MIN;
            x_end[1] = LV_COORD_MIN;
            x_end[2] = LV_COORD_MIN;
            x_end[3] = LV_COORD_MIN;
            int xe = 0;
            for (int16_t xi = -r_outer; xi < 0; xi++) {

                uint32_t r_act_sqr = xi * xi + yi * yi;
                if (r_act_sqr > r_out_sqr)
                    continue;

                deg_base = integer_atan(xi, yi) - 180;

                deg = 180 + deg_base;
                if (deg_test(deg, start_angle, end_angle)) {
                    if (x_start[0] == LV_COORD_MIN)
                        x_start[0] = xi;
                }
                else if (x_start[0] != LV_COORD_MIN && x_end[0] == LV_COORD_MIN) {
                    x_end[0] = xi - 1;
                }

                deg = 360 - deg_base;
                if (deg_test(deg, start_angle, end_angle)) {
                    if (x_start[1] == LV_COORD_MIN)
                        x_start[1] = xi;
                }
                else if (x_start[1] != LV_COORD_MIN && x_end[1] == LV_COORD_MIN) {
                    x_end[1] = xi - 1;
                }

                deg = 180 - deg_base;
                if (deg_test(deg, start_angle, end_angle)) {
                    if (x_start[2] == LV_COORD_MIN)
                        x_start[2] = xi;
                }
                else if (x_start[2] != LV_COORD_MIN && x_end[2] == LV_COORD_MIN) {
                    x_end[2] = xi - 1;
                }

                deg = deg_base;
                if (deg_test(deg, start_angle, end_angle)) {
                    if (x_start[3] == LV_COORD_MIN)
                        x_start[3] = xi;
                }
                else if (x_start[3] != LV_COORD_MIN && x_end[3] == LV_COORD_MIN) {
                    x_end[3] = xi - 1;
                }

                if (r_act_sqr < r_in_sqr) {
                    xe = xi;
                    // No need to continue the iteration in x
                    // once we found the inner edge of the arc
                    break;
                }
            }
            // Add +1 to all lengths as
            // LittlevGL H/V line functions' len param is actually len-1

            // top left quadrant
            if (x_start[0] != LV_COORD_MIN) {
                if (x_end[0] == LV_COORD_MIN)
                    x_end[0] = xe - 1;
                drawLineHoriz(x + x_start[0], y + yi, x_end[0] - x_start[0] + 1, color);
            }
            // bottom left quadrant
            if (x_start[1] != LV_COORD_MIN) {
                if (x_end[1] == LV_COORD_MIN)
                    x_end[1] = xe - 1;
                drawLineHoriz(x + x_start[1], y - yi, x_end[1] - x_start[1] + 1, color);
            }
            // top right quadrant
            if (x_start[2] != LV_COORD_MIN) {
                if (x_end[2] == LV_COORD_MIN)
                    x_end[2] = xe - 1;
                drawLineHoriz(x - x_end[2], y + yi, abs(x_end[2] - x_start[2]) + 1, color);
            }
            // bottom right quadrant
            if (x_start[3] != LV_COORD_MIN) {
                if (x_end[3] == LV_COORD_MIN)
                    x_end[3] = xe - 1;
                drawLineHoriz(x - x_end[3], y - yi, abs(x_end[3] - x_start[3]) + 1, color);
            }
        }
    }


    void drawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, T color)
    {
    	if (width == 0 || height == 0)
    		return;
    	surface_.fillLine(x, y, width, color);
    	surface_.fillLine(x, y + height - 1, width, color);
    	for (uint16_t row = y + height - 1; row >= y; row--) {
    		surface_.setPixel(x, row, color);
    		surface_.setPixel(x + width -1, row, color);
    	}
    }

    void drawRectSolid(uint16_t x, uint16_t y, uint16_t width, uint16_t height, T color)
    {
    	// TODO bounds check
    	if (width == 0 || height == 0)
    		return;
    	surface_.fillRect(x, y, width, height, color);
    }

    /** Draw a one pixel line */
    void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T color)
    {
    	if (y0 == y1) {
    		drawLineHoriz(x0, y0, x1 - x0, color);
			return;
    	}
    	if (x0 == x1) {
    		drawLineVert(x0, y0, y1 - y0, color);
    	}
        // TODO: types and document limits
        // Bresenham’s algo
        int16_t dx = abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = abs(y1 - y0);
        int16_t  sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        while (true) {
            surface_.setPixel(x0, y0, color);
            if (x0 == x1 && y0 == y1)
                break;
            e2 = err;
            if (e2 > -dx) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dy) {
                err += dx;
                y0 += sy;
            }
        }
    }

    /** Draw a one pixel horizontal line */
    void drawLineHoriz(int16_t x, int16_t y, int16_t length, T color)
    {
        if (y < 0 || y >= height_)
            return;
        // TODO: test
        if (length < 0) {
            x += length;
            length *= -1;
        }
        if (x < 0) {
            length += x;
            if (length <= 0)
                return;
            x = 0;
        }
    	surface_.fillLine(x, y, length, color);
    }

    /** Draw a one pixel vertical line */
    void drawLineVert(int16_t x, int16_t y, int16_t length, T color)
    {
        if (x < 0 || x >= width_)
            return;
        // TODO: test
        if (length < 0) {
            y += length;
            length *= -1;
        }
        if (y < 0) {
            length += y;
            if (length <= 0)
                return;
            y = 0;
        }
    	for (int16_t row = y + length - 1; row >= y; row--)
    		surface_.setPixel(x, row, color);
    }

    /**
     * Very approximate and inefficient algorithm. Need to fix diagonal lines.
     *
     * Warning: Uses >4*line height in stack space
     *
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param thickness
     * @param solid
     * @param color
     */
    void drawLineThick(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t thickness, bool solid, T color)
    {
    	if (thickness == 0)
    		return;
    	if (thickness == 1) {
    		drawLine(x0, y0, x1, y1, color);
    		return;
    	}
    	if (y0 == y1) { // horiz line
            uint16_t half_thick = thickness / 2;
            if (solid)
                drawRectSolid(std::min(x0, x1), y0 - half_thick, abs(x1 - x0), thickness, color);
            else
                drawRect(std::min(x0, x1), y0 - half_thick, abs(x1 - x0), thickness, color);
            return;
    	}
    	if (y1 < y0) { // drawing code requires y0 -> y1 be increasing
    		uint16_t temp = y0;
    		y0 = y1;
    		y1 = temp;
    	}
    	if (x0 == x1) { // vertical line
    		uint16_t half_thickness = thickness / 2;
    		if (solid)
    		    drawRectSolid(x0 - half_thickness, y0, thickness, y1 - y0, color);
    		else
    		    drawRect(x0 - half_thickness, y0, thickness, y1 - y0, color);
    		return;
    	}

        //int32_t dx = (x1 - x0);
        //int32_t dy = (y1 - y0);
        float dx = (x1 - x0);
        float dy = (y1 - y0);
        uint16_t line_len = sqrt(dx*dx + dy*dy);

        float half_thick = thickness / 2;
        dx *= half_thick;
        dy *= half_thick;

        dx /= line_len;
        dy /= line_len;

        Point corners[4];
        if (x1 > x0) {
			corners[0] = { static_cast<int16_t>(x0 - dy), static_cast<int16_t>(y0 + dx) };
			corners[1] = { static_cast<int16_t>(x0 + dy), static_cast<int16_t>(y0 - dx) };
			corners[2] = { static_cast<int16_t>(x1 + dy), static_cast<int16_t>(y1 - dx) };
			corners[3] = { static_cast<int16_t>(x1 - dy), static_cast<int16_t>(y1 + dx) };
        }
        else {
			corners[1] = { static_cast<int16_t>(x0 - dy), static_cast<int16_t>(y0 + dx) };
			corners[2] = { static_cast<int16_t>(x0 + dy), static_cast<int16_t>(y0 - dx) };
			corners[3] = { static_cast<int16_t>(x1 + dy), static_cast<int16_t>(y1 - dx) };
			corners[0] = { static_cast<int16_t>(x1 - dy), static_cast<int16_t>(y1 + dx) };
        }
        printf("[%d,%d], [%d,%d], [%d,%d], [%d,%d]\r\n", corners[0].x, corners[0].y, corners[1].x, corners[1].y, corners[2].x, corners[2].y, corners[3].x, corners[3].y);

        // Top down:
        // - left : 1->0, 0->3
        // - right: 1->2, 2->3
        if (solid) {
            uint16_t y_start = std::max(zero_, corners[1].y);
            uint16_t y_stop = std::min(height_, corners[3].y);
            int16_t height = y_stop - y_start; // line is always increasing y

            // TODO: VLA
            uint16_t left_edge[height + 1] = {0};
            uint16_t right_edge[height + 1] = {0};
            // Calculate the left/right edges of the shape
            drawLineArray(corners[1].x, corners[1].y, corners[0].x, corners[0].y, y_start, left_edge); // left edge
            drawLineArray(corners[0].x, corners[0].y, corners[3].x, corners[3].y, y_start, left_edge); // left edge
            drawLineArray(corners[1].x, corners[1].y, corners[2].x, corners[2].y, y_start, right_edge); // right edge
            drawLineArray(corners[2].x, corners[2].y, corners[3].x, corners[3].y, y_start, right_edge); // right edge
            uint16_t i = 0;
            for (int16_t y = y_start; y <= y_stop; y++) {
                drawLineHoriz(left_edge[i], y, right_edge[i] - left_edge[i] + 1, color);
                i++;
            }
        }
        else {
            drawLine(corners[1].x, corners[1].y, corners[0].x, corners[0].y, color); // top left edge
            drawLine(corners[0].x, corners[0].y, corners[3].x, corners[3].y, color); // bottom left edge
            drawLine(corners[1].x, corners[1].y, corners[2].x, corners[2].y, color); // top right edge
            drawLine(corners[2].x, corners[2].y, corners[3].x, corners[3].y, color); // bottom right edge
        }
    }

    /* TODO mapping needs to be done at the surface level to avoid allocation
     * void drawBitmap(int16_t x, int16_t y, const Image2d& image, Align align, T color_map[])
    {
    }*/

    void drawBitmap(int16_t x, int16_t y, const Image2d& image, Align align)
    {

        Rect16 r = { image.width, image.height };
        adjustPoint(x, y, r, align);
        // TODO: matching bpp
        surface_.copyRect(x, y, image.width, image.height, image.data);
    }

    // http://public.callutheran.edu/~reinhart/CSC505/Week1/BresenhamCircles.pdf
    // //
    void drawArcTesting(uint16_t x, uint16_t y, uint16_t radius, T color)
    {
        // Bresenham’s Algorithm
        int16_t d = 1 - radius;
        int16_t xpos = radius;
        int16_t ypos = 0;

        uint16_t start_ang = 20;
        uint16_t arc_len = 240;

        do {
            if (start_ang == 0 && arc_len >= 45)
                surface_.setPixel(x + ypos, y - xpos, color); //   0 ->  45

            if (start_ang <= 45 && start_ang + arc_len >= 90)
                surface_.setPixel(x + xpos, y - ypos, color); //  45 ->  90

            if (start_ang <= 90 && start_ang + arc_len >= 135)
                surface_.setPixel(x + xpos, y + ypos, color); //  90 -> 135

            if (start_ang <= 135 && start_ang + arc_len >= 180)
                surface_.setPixel(x + ypos, y + xpos, color); // 135 -> 180

            if (start_ang <= 180 && start_ang + arc_len >= 225)
                surface_.setPixel(x - ypos, y + xpos, color); // 180 -> 225

            if (start_ang <= 225 && start_ang + arc_len >= 270)
                surface_.setPixel(x - xpos, y + ypos, color); // 225 -> 270

            if (start_ang <= 270 && start_ang + arc_len >= 315)
                surface_.setPixel(x - xpos, y - ypos, color); // 270 -> 315

            if (start_ang <= 315 && start_ang + arc_len >= 360)
                surface_.setPixel(x - ypos, y - xpos, color); // 315 -> 360
/*          if (ratio > xy_start && ratio < xy_stop) {

                surface_.setPixel(x + ypos, y - xpos, color); //   0 ->  45
                surface_.setPixel(x + xpos, y - ypos, color); //  45 ->  90

                surface_.setPixel(x + xpos, y + ypos, color); //  90 -> 135
                surface_.setPixel(x + ypos, y + xpos, color); // 135 -> 180

                surface_.setPixel(x - ypos, y + xpos, color); // 180 -> 225
                surface_.setPixel(x - xpos, y + ypos, color); // 225 -> 270

                surface_.setPixel(x - xpos, y - ypos, color); // 270 -> 315
                surface_.setPixel(x - ypos, y - xpos, color); // 315 -> 360
            }*/
            ypos++;
            d = (d < 0)
                ? d + 2 * ypos + 1
                : d + 2 * (ypos - --xpos) + 1;
        }
        while (xpos >= ypos);
    }

    // http://downloads.gamedev.net/pdf/gpbb/gpbb35.pdf
    // https://www.gamedev.net/tutorials/_/technical/graphics-programming-and-theory/graphics-programming-black-book-r1698/
    // TODO: drawArc
    // https://github.com/littlevgl/lvgl/issues/252

    void drawArc2(float cx, float cy, float px, float py, float theta, int N)
    {
        float dx = px - cx;
        float dy = py - cy;
        float r2 = dx * dx + dy * dy;
        float r = sqrt(r2);
        float ctheta = cos(theta/(N-1));
        float stheta = sin(theta/(N-1));
        surface_.setPixel(cx + dx, cy + dy);
        for(int i = 1; i != N; ++i)
        {
            float dxtemp = ctheta * dx - stheta * dy;
            dy = stheta * dx + ctheta * dy;
            dx = dxtemp;
            for(int i = 1; i != N; ++i)
                surface_.setPixel(cx + dx, cy + dy);
        }
    }
    //https://stackoverflow.com/a/27756701/1601122

private:
    struct Point {
    	int16_t x;
    	int16_t y;
    };

    IDrawingSurface<T>& surface_;
    static constexpr int16_t zero_ = 0;
    const int16_t height_ = surface_.getHeight();
    const int16_t width_ = surface_.getWidth();

    /**
     * Populate an array with a line's x coordinates. The array index is the y
     * coordinate of the line relative to the line's origin.
     *
     * e.g. Line with points (10,12) (11,13) (11,14) (12,15)
     *      will generate x_coord = { 10, 11, 11, 12 }
     *
     * y0 -> y1 must be increasing.
     *
     * @param x0
     * @param y0
     * @param x1
     * @param y1 must be >= y0
     * @param y_origin
     * @param buf
     */
    void drawLineArray(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t y_origin, uint16_t* x_coord)
    {
        int16_t dx = abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = y1 - y0; // Always increasing y
        int err = (dx > dy ? dx : -dy) / 2, e2;
        while (true) {
        	// TODO: what about multiple x per y
            if (y0 >= 0 && y0 < height_)
            	// constrain to surface bounds
                // TODO: shouldn't this be width_ - 1?
                x_coord[y0 - y_origin] = std::max(std::min(x0, width_), zero_);
            if (x0 == x1 && y0 == y1)
                break;
            e2 = err;
            if (e2 > -dx) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dy) {
                err += dx;
                y0++;
            }
        }
    }
};

} // namespace Libp

#endif /* PAINTERCOLOR_H_ */
