/* Copyright (c) 2005 Arachi, Inc. and Stanford University. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <tao/matrix/TaoDeMath.h>

/* 
 * y = LU x
 * find x given LU and y 
 * diag(U) = [1 1 1 ... 1]
 */
void deVector6::backSub(const deMatrix6& lu, const deVector6& y)
{
	/* LU x = y ----> x */
	/* L d  = y --> fw sub */
	/* U x = d ---> bw sub */

	deInt i, j;
	deFloat d[6];

	/* forward */
	for (i = 0; i < 6; i++)
	{
		d[i] = y.elementAt(i);
		for (j = 0; j < i; j++)
			d[i] -= lu.elementAt(i, j) * d[j];
		d[i] /= lu.elementAt(i, i);
	}
	/* backward */
	for (i = 5; i >= 0; i--)
	{
		elementAt(i) = d[i];
		for (j = i + 1; j < 6; j++)
			elementAt(i) -= lu.elementAt(i, j) * elementAt(j);
	}
}

/*
 * y = LU x
 * y = LL' x
 * find x given LU and y 
 * (symmetric and positive definite)
 */
void deVector6::backSubSPD(const deMatrix6& lu, const deVector6& y)
{
	/* U = L' */
	/* LU = LL' */
	/* LU x = y ----> x */
	/* L d  = y --> fw sub */
	/* U x = d ---> bw sub */

	deInt i, j;
	deFloat d[6];

	/* forward */
	for (i = 0; i < 6; i++)
	{
		d[i] = y.elementAt(i);
		for (j = 0; j < i; j++)
			d[i] -= lu.elementAt(i, j) * d[j];
		d[i] /= lu.elementAt(i, i);
	}
	/* backward */
	for (i = 5; i >= 0; i--)
	{
		elementAt(i) = d[i];
		for (j = i + 1; j < 6; j++)
			elementAt(i) -= lu.elementAt(j, i) * elementAt(j);
		elementAt(i) /= lu.elementAt(i, i);
	}
}
