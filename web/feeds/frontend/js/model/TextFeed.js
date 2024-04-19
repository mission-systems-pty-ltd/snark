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

/**
 * Created by vrushali on 15/10/15.
 */

define('TextFeed', ["jquery", "Feed"], function ($) {
    var Feed = require('Feed');
    var TextFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        this.base(feed_name, feed_path, config);
        this.height = this.config.height == undefined ? 0 : this.config.height;
        this.width = this.config.width == undefined ? 0 : this.config.width; // todo: this.width = this.config.width == undefined ? 0 : this.config.width;
    };
    TextFeed.prototype = Object.create(Feed.prototype);
    TextFeed.prototype.load = function () {
        $.ajax({
            crossDomain: true,
            context: this,
            url: this.get_url(),
            timeout: globals.timeout
        }).done(function (data, textStatus, jqXHR) {
            this.onload(data);
        }).fail(function (jqXHR, textStatus, errorThrown) {
            this.onerror();
        });
        if( this.height > 0 ) { this.target.height( this.height ); }
        // todo: if( this.width > 0 ) { this.target.width( this.width ); }
    };
    TextFeed.prototype.onload_ = function (data)
    {
        if (data && data.length && data[data.length - 1] == '\n') { data = data.substring(0, data.length - 1); }
        var orig_data = data;
        data = data ? data : '&nbsp;';
        if (this.form_show_buttons) {
            var panel = $(this.target).find(".text-pre");
            if (panel.length > 0)
            {
                $(panel).html(data);
            }
            else
            {
                this.target.append('<tr><td><pre class="text-pre">' + data + '</pre></td></tr>');
                panel = $(this.target).find(".text-pre");
            }
            if (orig_data)
            {
                $(panel).css("overflow-y", "scroll");
            }
            else
            {
                $(panel).css("overflow", "auto");
            }
            var this_ = this;
            var height = this.height > 0 ? this.height : 100;
            // todo: var width = this.width > 0 ? this.width : 150;
            $(panel).css("max-height", function () { return height; });
            this.draw();
            $(panel).scrollTop($(panel)[0].scrollHeight);
        }
        else
        {
            this.target.append('<tr><td><pre ' + ( this.config.style == undefined ? '' : 'style="' + this.config.style + '"' ) + '>' + data + '</pre></td></tr>');
            //this.target.append( '<tr><td><pre' + style + '>' + data + '</pre></td></tr>' );
            this.draw();
        }
    };
    TextFeed.prototype.draw = function () {
        while (this.target.find('tbody tr').length > this.config.text.show_items) {
            this.target.find('tbody tr').first().remove();
        }
    };
    return TextFeed;
});
