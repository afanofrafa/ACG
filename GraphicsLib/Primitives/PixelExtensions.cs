using System.Windows;
using System.Windows.Media.Imaging;

namespace GraphicsLib.Primitives {
    public static class PixelExtensions {

        private static Int32Rect defaultRect = new Int32Rect(0, 0, 1, 1);

        public static void BeginLock(this WriteableBitmap bitmap) {
            bitmap.Lock();
        }

        public static unsafe void SetPixelLocked(this WriteableBitmap bitmap, int x, int y, uint argb) {
            unsafe {
                uint* ptr = (uint*)bitmap.BackBuffer;
                ptr[y * bitmap.PixelWidth + x] = argb;
            }
            defaultRect.X = x;
            defaultRect.Y = y;
            bitmap.AddDirtyRect(defaultRect);
        }
        public static unsafe void SetPixelLockedNoDirty(this WriteableBitmap bitmap, int x, int y, uint argb) {
            unsafe {
                uint* ptr = (uint*)bitmap.BackBuffer;
                ptr[y * bitmap.PixelWidth + x] = argb;
            }
        }
        public static unsafe void SetPixelLockedNoDirty(this WriteableBitmap bitmap, int width, int x, int y, uint argb) {
            unsafe {
                uint* ptr = (uint*)bitmap.BackBuffer;
                ptr[y * width + x] = argb;
            }
        }

        public static void EndLock(this WriteableBitmap bitmap) {
            bitmap.Unlock();
        }

        public static void SetPixel(this WriteableBitmap bitmap, int x, int y, uint color) {
            defaultRect.X = x;
            defaultRect.Y = y;
            bitmap.WritePixels(defaultRect, BitConverter.GetBytes(color), 4, 0);
        }
        const int INSIDE = 0;
        const int LEFT = 1;
        const int RIGHT = 2;
        const int TOP = 4;
        const int BOTTOM = 8;
        /// <summary>
        /// Encodes position of point outside of the rectangle into a 4-bit code.
        /// top is above screen (y < 0)
        /// botton is below screen (y > ymax)
        /// left is to the left of screen (x < 0)
        /// right is to the right of screen (x > xmax)
        /// </summary>
        static int ComputeOutCode(int x, int y, int xmax, int ymax) {

            int code = INSIDE;

            if (x < 0)
                code |= LEFT;
            else if (x > xmax)
                code |= RIGHT;
            if (y < 0)
                code |= TOP;
            else if (y > ymax)
                code |= BOTTOM;

            return code;
        }
        public static void DrawLine(this WriteableBitmap bitmap, int bitmapWidth, int bitmapHeight, int x0, int y0, int x1, int y1, uint color) {
            int xmax = bitmapWidth - 1;
            int ymax = bitmapHeight - 1;
            // Cohen–Sutherland algorithm to cut line from outside the rectangle to inside.
            int outcode0 = ComputeOutCode(x0, y0, xmax, ymax);
            int outcode1 = ComputeOutCode(x1, y1, xmax, ymax);
            while (true) {
                //both points are inside of rectangle
                if ((outcode0 | outcode1) == 0) {
                    break;
                }
                //both points are on the same side outside of rectangle
                //and line doesnt intersect with rectangle
                if ((outcode0 & outcode1) != 0) {
                    return;
                }
                int outcodeOut = outcode0 != 0 ? outcode0 : outcode1;
                int x;
                int y;
                //find point (interpolant) that intersects line and correspoding side of rectangle
                //   +
                //    \
                //  +--+-----+
                //  |   \    |
                //  |    +   |
                //
                //  casts to long to avoid overflow while multiplication
                if ((outcodeOut & BOTTOM) != 0) {
                    y = ymax;
                    x = (int)(x0 + (long)(x1 - x0) * (ymax - y0) / (y1 - y0));
                } else if ((outcodeOut & TOP) != 0) {
                    y = 0;
                    x = (int)(x0 + (long)(x1 - x0) * (0 - y0) / (y1 - y0));
                } else if ((outcodeOut & RIGHT) != 0) {
                    x = xmax;
                    y = (int)(y0 + (long)(y1 - y0) * (xmax - x0) / (x1 - x0));
                } else {
                    x = 0;
                    y = (int)(y0 + (long)(y1 - y0) * (0 - x0) / (x1 - x0));
                }
                if (outcodeOut == outcode0) {
                    x0 = x;
                    y0 = y;
                    outcode0 = ComputeOutCode(x0, y0, xmax, ymax);
                } else {
                    x1 = x;
                    y1 = y;
                    outcode1 = ComputeOutCode(x1, y1, xmax, ymax);
                }
            }
            // Bresenham algorithm to draw line inside the rectangle.
            unsafe {
                uint* ptr = (uint*)bitmap.BackBuffer;

                int dx = x1 > x0 ? x1 - x0 : x0 - x1;
                int dy = y1 > y0 ? y1 - y0 : y0 - y1;
                int directionX = x1 >= x0 ? 1 : -1;
                int directionY = y1 >= y0 ? 1 : -1;

                if (dy < dx) {
                    if (x1 < x0) {
                        // swap points so drawing is from left to right
                        (x0, x1) = (x1, x0);
                        (y0, y1) = (y1, y0);
                        directionY = -directionY;
                    }

                    int error = 0;
                    int dError = dy * 2;
                    int maxError = dx * 2;
                    int y = y0;
                    for (int x = x0; x <= x1; x++) {
                        ptr[y * bitmapWidth + x] = color;
                        error += dError;
                        if (error > 1) 
                        {
                            y += directionY;
                            error -= maxError;
                        }
                    }
                } else {
                    // swap points so drawing is from top to bottom
                    if (y1 < y0) {
                        (x0, x1) = (x1, x0);
                        (y0, y1) = (y1, y0);
                        directionX = -directionX;
                    }
                    int error = 0;
                    int dError = dx * 2;
                    int maxError = dy * 2;
                    int x = x0;
                    for (int y = y0; y <= y1; y++) {
                        ptr[y * bitmapWidth + x] = color;
                        error += dError;
                        if (error > 1)
                        { 
                            x += directionX;
                            error -= maxError;
                        }
                    }
                }
            }
        }

    }
}
