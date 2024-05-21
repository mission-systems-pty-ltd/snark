// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

define(['jquery', 'Feed', 'Grid'], function ($, Feed, Grid)
{
    var ImageFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        this.base(feed_name, feed_path, config);
        this.popup_img = $("#popup" + this.feed_name + " img");
        this.init();
    };
    ImageFeed.prototype = Object.create(Feed.prototype);
    ImageFeed.prototype.init = function ()
    {
        this.img = $('<img>').appendTo(this.target);
        this.loader = new Image();
        var _this = this;
        this.loader.onload = function ()
        {
            clearTimeout(_this.timer);
            _this.onload(this.src);
        };
        this.loader.onerror = function ()
        {
            clearTimeout(_this.timer);
            _this.onerror();
            this.target.width( this.width );
            this.target.height( this.height );
        }
    };
    ImageFeed.prototype.load = function ()
    {
        this.loader.width_prev = this.loader.width;
        this.loader.height_prev = this.loader.height;
        this.loader.src = this.get_url();
        this.timer = setTimeout(this.loader.onerror, globals.timeout);
    };
    ImageFeed.prototype.onload_ = function (data)
    {
        this.img.attr('src', data);
        this.popup_img.attr('src', data);
        // this.popup_img.attr('width', this.loader.width);
        // this.popup_img.attr('height', this.loader.height);
        if( this.loader.width != this.loader.width_prev || this.loader.height != this.loader.height_prev )
        {
            if( this.target.resizable( 'instance' ) ) { this.target.resizable('destroy'); }
            var minHeight = 269;
            if( globals.isMobile )
            {
                var maxWidth = ((window.innerWidth > 0) ? window.innerWidth : screen.width ) - 100;
                var width = this.loader.width;
                var height = this.loader.height;
                if( width > maxWidth )
                {
                    var ratio = maxWidth / width;   // get ratio for scaling image
                    height = parseInt(height * ratio);    // Reset height to match scaled image
                    width = parseInt(width * ratio);    // Reset width to match scaled image
                }
                this.target.width( width );
                if( height < minHeight ) { minHeight = height; }
                this.target.height(height);
                $(this.target).css( "min-height", function () { return minHeight; } );
            }
            else
            {
                if( this.config.ignore_size_limits ) // todo! document!
                {
                    var width = this.loader.width;
                    var height = this.loader.height;
                    this.target.width(width);
                    this.target.height(height);
                    if( height < minHeight ) { minHeight = height; } // quick and dirty; todo: are we breaking something?
                    // todo? min/max width/height
                }
            }
            this.target.resizable({
                aspectRatio: true,
                autoHide: true,
                minWidth: 269,
                minHeight: minHeight
            });
            this.remove_grid();
        }
        if( this.config.grid && this.config.grid.show && !this.grid ) { this.add_grid(); }
    };
    ImageFeed.prototype.add_grid = function ()
    {
        this.grid = new Grid(this.target, this.config.grid, this.loader.width, this.loader.height);
    };
    ImageFeed.prototype.remove_grid = function ()
    {
        if( this.grid ) { this.grid.remove(); delete this.grid; }
    };
    ImageFeed.prototype.draw_grid = function ()
    {
        if( this.grid ) { this.grid.draw(); }
    };
    return ImageFeed;
});
