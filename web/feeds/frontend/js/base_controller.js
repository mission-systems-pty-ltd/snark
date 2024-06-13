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

/// @authors Vinny Do, Vrushali Satpute

//---------------------------------------------//
//
// Global Variables
//
// --------------------------------------------//

var feed;
var feeds = {};
var pending = {};
var gui;
var config_files = [];

define( 'base_controller'
      , [ 'jquery'
        , "jquery_timeago"
        , "bootstrap"
        , "ol"
        , 'utils'
        , "Feed"
        , "CsvFeed"
        , 'TextFeed'
        , 'ImageFeed'
        , 'GraphFeed'
        , 'ImageStreamFeed'
        , 'TrackFeed'
        , 'TrackOptions'
        , 'MapFeed'
        , 'StartStopFeed'
        , 'MapOptions'
        , 'GridOptions']
      , function ($)
{
    var Feed = require('Feed');
    // var TextFeed = require('TextFeed');
    var TrackOptions = require('TrackOptions');
    var GridOptions = require('GridOptions');
    var MapOptions = require('MapOptions');
    // var GraphFeed = require('GraphFeed');
    // var ImageStreamFeed = require('ImageStreamFeed');
    // var CsvFeed = require('CsvFeed');
    // var TrackFeed = require('TrackFeed');
    // var TrackOptions = require('TrackOptions');
    // var MapOptions = require('MapOptions');
    // var GridOptions = require('GridOptions');
    var load_config_fn;
    var create_feed_fn;
    var current_config_file;
    var container_width;
    var is_mobile = false;
    var query_string = parse_query_string();
    var base_controller = function (container_w, mobile)
    {
        container_width = container_w;
        is_mobile = mobile;
        if (this.constructor === base_controller) { throw new Error("Can't instantiate abstract class!"); }
    };
    base_controller.prototype.add_poll_body = function (feed_name, element, fields, popup_div)
    {
        var id = '#' + feed_path_to_id(feed_name);
        var class_str = globals.isMobile ? '' : ' hideable transparent';
        var input_fields_link = feed_name + "-fields";
        var content_html = "";
        if (fields != undefined)
        {
            // content_html += '<a data-toggle="collapse" class="text-center" href="#' + input_fields_link + '">Input fields</a>'
            //     + '<div class="inputs-group collapse" id="' + input_fields_link + '"></div>';
            //content_html += '<div class="inputs-group " id="' + input_fields_link + '"></div>';
        }
        $(id).append(
            '<h3>' + feed_name +
            '  <button class="panel-refresh" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: auto refresh"><span class="status text-muted glyphicon glyphicon-stop"></span></button>' +
            '  <button class="panel-settings' + class_str + '" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button>' +
            '  <button class="panel-compact' + class_str + '" title="compact"><span class="text-muted glyphicon glyphicon-resize-small"></span></button>' +
            '</h3>');
        $(id).append('<div class="panel-body">' +
            '  <time class="timestring time small">&nbsp;</time>' +
            '  <time class="timeago time small">&nbsp;</time>' +
            '<div class="target-container">' +
            content_html + element + '</div></div>'
        );
        if (popup_div != undefined)
        {
            $(id).append(popup_div);
            if (globals.isMobile) { $("#popup" + feed_name).popup(); }
        }
    };
    base_controller.prototype.initialize_gui = function (width)
    {
        globals.stop();
        feeds = {};
        pending = {};
        current_config_file = globals.config_file;
        $('#container').empty();
        try { $('#container').sortable('destroy'); } catch (e) {}
        try { $('#container').resizable('destroy'); } catch (e) {}
        if (gui) { gui.destroy(); }
        gui = new dat.GUI({ width: width });
    };
    base_controller.prototype.initialize_load_config = function ()
    {
        if (!current_config_file) { return; }
        document.title = " Feeds - " + current_config_file.substr(current_config_file.indexOf("/") + 1);
        load_gui_config(current_config_file);
        load_layout(current_config_file);
        for (var id in feeds)
        {
            var feed_obj = feeds[id];
            if (feed_obj == undefined) { continue; }
            if (!feed_obj.el.is(':visible')) { continue; }
            if (feed_obj.config.type == 'stream') { feed_obj.start(); } else { feed_obj.refresh(); }
        }
        $('#container').sortable({
            items: 'li.panel',
            opacity: 0.8,
            placeholder: 'panel panel-placeholder',
            cursor: 'move',
            start: function (event, ui) {
                ui.placeholder.width(ui.item.width());
                ui.placeholder.height(ui.item.height());
            }
        }).resizable({
            handles: 'e'
        });

    };
    base_controller.prototype.init_load_config = function (load_config, create_feed)
    {
        load_config_fn = load_config;
        create_feed_fn = create_feed;
        $.ajax({
            url: globals.config_dir
        }).done(function (data, textStatus, jqXHR) {
            $(data).find('a').each(function (index) {
                var href = $(this).attr('href');
                if (href.match(/[.]json$/)) {
                    config_files.push(globals.config_dir + '/' + href);
                }
            });
            if ((config_files.length > 0) && config_files.indexOf(globals.config_file) < 0) {
                globals.config_file = config_files[0];
            }
        }).fail(function (jqXHR, textStatus, errorThrown) {
        }).always(function (data_jqXHR, textStatus, errorThrown) {
            var file = query_string['config'];
            if (file)
            {
                globals.config_file = file;
            }
            else
            {
                var config_file = load_last_config_file();
                if (config_file && config_files.indexOf(config_file) != -1) { globals.config_file = config_file; }
            }
            if (config_files.indexOf(globals.config_file) == -1) { config_files.push(globals.config_file); }
            load_config_fn(globals.config_file);
        });
    };
    base_controller.prototype.config_url = function (file)
    {
      // Construct a config url for the given file, with timestamped query
      // param for cache busting.
      var timestamp = new Date().getTime();
      var url = file + '?_=' + timestamp;
      return url;
    }
    base_controller.prototype.add_gui_globals = function ()
    {
        gui.add(globals, 'config_file', config_files).name('config file').onFinishChange(function (value) { load_config_fn(globals.config_file); });
        var folder = gui.addFolder('globals');
        folder.add(globals, "refresh");
        folder.add(globals, "start");
        folder.add(globals, "stop");
        folder.add(globals, "show");
        folder.add(globals, "compact");
        folder.add(globals, "hide");
        folder.add(globals, "save");
        folder.add(globals, "reset");
        folder.add(globals, "enable_alerting").name("enable alerting");
        folder.add(globals, "disable_alerting").name("disable alerting");
        folder.add(globals, "clear_alerts").name("clear alerts");
        folder.add(globals, "alert_beep").name("alert beep");
    };
    base_controller.prototype.load_feed_items = function (frontend_config, feeds, path)
    {
        if( frontend_config.timeout ) { globals.timeout = frontend_config.timeout; }
        var is_host_specified = frontend_config.host != undefined;
        if( is_host_specified ) { globals.host = frontend_config.host; }
        var is_port_specified = frontend_config.port != undefined;
        if( is_port_specified ) { globals.port = frontend_config.port; }
        if( !is_host_specified && !is_port_specified )
        {
            $('#container').empty();
            $('#container').append('<p>Please specify either Host or Port in frontend configuration</p>');
            $('#container').append('<p>see: <a href="readme.txt">readme.txt</a></p>');
            $('#container').append('<p>see: <a href="examples/web.frontend.json">examples/web.frontend.json</a></p>');
            return;
        }
        // var feeds = frontend_config.feeds;
        if( path.length != 0 ) { path = path + "/"; }
        for( var feed_name in feeds )
        {
            //console.log( "==> feeds: " + feeds );
            var config = feeds[feed_name];
            if( Array.isArray( config ) ) { load_feed_array( frontend_config, config, path + feed_name ); }
            else { add_new_feed(frontend_config, config, path + feed_name); }
        }
    };
    base_controller.prototype.add_stream_body = function (feed_name, element)
    {
        var id = '#' + feed_path_to_id(feed_name);
        var class_str = globals.isMobile ? '' : ' hideable transparent';
        $(id).append(
            '<h3>' + feed_name +
            '  <button class="panel-stream-control" title="<kbd>click</kbd>: refresh<br><kbd>shift+click</kbd>: start/stop"><span class="status text-muted glyphicon glyphicon-stop"></span></button>' +
            '  <button class="panel-settings' + class_str + '" title="settings"><span class="text-muted glyphicon glyphicon-cog"></span></button></h3>' +
            '<div class="panel-body">' +
            element +
            '</div>'
        );
    };
    base_controller.prototype.init_actions = function ()
    {
        $('.map').hover( function () { $('#container').sortable('disable') }
                       , function () { $('#container').sortable('enable') });
        $('.panel').on('mouseover', function ()
        {
            $(this).find('h3').find('.hideable').removeClass('transparent');
            $(this).find('h3').find('div button').addClass('button-shadow');
        });
        $('.panel').on('mouseout', function ()
        {
            $(this).find('h3').find('.hideable').addClass('transparent');
            $(this).find('h3').find('div button').removeClass('button-shadow');
        });
        $('.panel button').tooltip({html: true});
        $('.panel-refresh').on('click', function (event)
        {
            var id = $(this).closest('li').attr('id');
            if (event.shiftKey) { gui.toggleProperty('auto', id); } else { feeds[id].refresh(); }
        });
        $('.panel-stream-control').on('click', function (event)
        {
            var id = $(this).closest('li').attr('id');
            if (event.shiftKey) { feeds[id].toggle(); } else { feeds[id].refresh(); }
        });
        $('.panel-settings').on('click', function (event)
        {
            if (globals.isMobile)
            {
                var gui_name = $(this).closest('li').attr('data-name');
                gui.collapseAllFolders();
                gui.open();
                gui.openFolder(gui_name);
            }
            else
            {
                var gui_name = $(this).closest('li').data('name');
                $.each(gui.__folders, function (index, folder) { folder.close(); });
                gui.__folders[gui_name].open();
                gui.open();
            }
        });
        $('.panel-close').on('click', function (event)
        {
            var id = $(this).closest('li').attr('id');
            var feed_obj = feeds[id];
            if (feed_obj.config.type == 'stream') { gui.setProperty('show', false, feed_obj.feed_name); }
            else { gui.setProperty('view', 'hide', feed_obj.feed_name); }
        });
        $('.panel-compact').on('click', function (event)
        {
            var id = $(this).closest('li').attr('id');
            var feed_obj = feeds[id];
            if (feed_obj.config.type != 'stream') { gui.setProperty('view', feed_obj.config.view == 'compact' ? 'show' : 'compact', feed_obj.feed_name); }
        });
        $('.timeago').on('mouseenter', function (e)
        {
            var id = $(this).closest('li').attr('id');
            var feed_obj = feeds[id];
            feed_obj.timeago.hide();
            feed_obj.timestring.show();
        });
        $('.timestring').on('mouseleave', function (e)
        {
            var id = $(this).closest('li').attr('id');
            var feed_obj = feeds[id];
            feed_obj.timestring.hide();
            feed_obj.timeago.show();
        });
        $('.panel').on('mouseleave', function (e)
        {
            var id = $(this).closest('li').attr('id');
            var feed_obj = feeds[id];
            if (feed_obj.timeago)
            {
                feed_obj.timestring.hide();
                feed_obj.timeago.show();
            }
        });
        $(document).on('keydown', function (event) { if (event.ctrlKey) { toggle_sortable(false); } });
        $(document).on('keyup', function (event) { toggle_sortable(true); });
        $(window).on('focusout', function (event) { toggle_sortable(true); });
        $(window).on('beforeunload', function (e) { save_last_config_file(current_config_file); });
    };
    function load_layout(config_file)
    {
        var key = 'feeds.layout[' + config_file + ']';
        var layout = JSON.parse(localStorage.getItem(key));
        if (!layout) { return; }
        layout.reverse();
        layout.forEach(function (value, index)
        {
            var container = $('#container');
            if (value.id === 'container') { container.width(value.width); return; }
            var panel = $('#' + value.id);
            if (!panel.length) { return; }
            container.prepend(panel);
            var feed = feeds[value.id];
            if (feed != undefined)
            {
                if ('width' in value) { feed.target.width(value.width); }
                if ('height' in value) { feed.target.height(value.height); }
            }
        });
    }
    function load_gui_config(config_file)
    {
        var key = 'feeds.gui.config[' + config_file + ']';
        var config = JSON.parse(localStorage.getItem(key));
        if (!config) { return false; }
        gui.setProperty('alert_beep', config['globals'].alert_beep, 'globals');
    }

    function get_feed_name(element) { return feed_path_to_id($(element).closest('ul').find('li.title').first().text()); }

    function add_gui_track_options(folder, config)
    {
        var track_folder = folder.addFolder('track options');
        $(track_folder.domElement).closest('li.folder').find('li.title').first().addClass('subfolder');
        track_folder.add(config, 'image').onFinishChange(function (value)
        {
            var feed_name = get_feed_name(track_folder.domElement);
            feeds[feed_name].set_background();
        });
        track_folder.add(config, 'extent').onFinishChange(function (value)
        {
            var feed_name = get_feed_name(track_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.set_extent();
        });
        track_folder.add(config, 'scale', 1, 300).name('scale (%)').step(0.01).onChange(function (value)
        {
            var feed_name = get_feed_name(track_folder.domElement);
            if (value < feeds[feed_name].min_scale()) { return; }
            feeds[feed_name].resize();
        });
        track_folder.add(config, 'trail').onChange(function (value)
        {
            var feed_name = get_feed_name(track_folder.domElement);
            if (!value) { feeds[feed_name].remove_trail(); }
        });
        track_folder.add(config, 'draw_interval', 1, 1000).name('draw interval (ms)').step(10).onChange(function (value)
        {
            var feed_name = get_feed_name(track_folder.domElement);
            feeds[feed_name].reset_draw_interval();
        });
        track_folder.add(config, 'alpha_step', 0, 0.9).name('alpha step').step(0.01);
        track_folder.add(config, 'radius', 0.5, 20).step(0.1);
        track_folder.addColor(config, 'fill');
        track_folder.addColor(config, 'stroke');
        track_folder.add(config, 'stroke_width', 0, 5).name('stroke width').step(0.5);
    }
    function add_gui_map_options(folder, config)
    {
        var map_folder = folder.addFolder('map options');
        $(map_folder.domElement).closest('li.folder').find('li.title').first().addClass('subfolder');
        map_folder.add(config, 'imagery_set', ['', 'Aerial', 'AerialWithLabels', 'Road']).onFinishChange(function (value)
        {
            var feed_name = get_feed_name(map_folder.domElement);
            var feed_obj = feeds[feed_name];
            if (value) { feed_obj.set_base_tile(); } else { feed_obj.set_base_layer(); }
        });
        map_folder.add(config, 'bing_maps_key').name('bing maps key').onFinishChange(function (value)
        {
            var feed_name = get_feed_name(map_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.set_base_tile();
        });
        map_folder.add(config, 'image').onFinishChange(function (value)
        {
            var feed_name = get_feed_name(map_folder.domElement);
            feeds[feed_name].set_base_layer();
        });
        map_folder.add(config, 'extent').onFinishChange(function (value)
        {
            var feed_name = get_feed_name(map_folder.domElement);
            feeds[feed_name].set_base_layer();
        });
        map_folder.add(config, 'follow');
        map_folder.add(config, 'trail');
        map_folder.add(config, 'draw_interval', 1, 1000).name('draw interval (ms)').step(10).onChange(function (value)
        {
            var feed_name = get_feed_name(map_folder.domElement);
            feeds[feed_name].reset_draw_interval();
        });
        map_folder.add(config, 'alpha_step', 0, 0.9).name('alpha step').step(0.01);
        map_folder.add(config, 'radius', 0.5, 20).step(0.1);
        map_folder.addColor(config, 'fill');
        map_folder.addColor(config, 'stroke');
        map_folder.add(config, 'stroke_width', 0, 5).name('stroke width').step(0.5);
    }
    function save_gui_config(config_file)
    {
        var key = 'feeds.gui.config[' + config_file + ']';
        var config = { globals: globals };
        for (var id in feeds) { config[id] = feeds[id].config; }
        localStorage.setItem(key, JSON.stringify(config));
    }
    function save(config_file)
    {
        if (!config_file) { return; }
        save_layout(config_file);
        save_gui_config(config_file);
    }
    function add_gui_grid_options(folder, config)
    {
        var grid_folder = folder.addFolder('grid options');
        grid_folder.add(config, 'show').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            value ? feed_obj.add_grid() : feed_obj.remove_grid();
        });
        grid_folder.add(config, 'axis_width', 0.5, 20).step(0.5).name('axis width').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        grid_folder.add(config, 'step_length', 0.5, 20).step(0.5).name('step length').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        grid_folder.add(config, 'x_offset', -100, 100).step(1).name('x offset (pixels)').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        grid_folder.add(config, 'y_offset', -100, 100).step(1).name('y offset (pixels)').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        var x_folder = grid_folder.addFolder('x-axis options');
        x_folder.add(config.x, 'min', -100, 100).step(1).onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        x_folder.add(config.x, 'max', -100, 100).step(1).onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        x_folder.add(config.x, 'step', 0, 100).step(1).onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        x_folder.addColor(config.x, 'color').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        var y_folder = grid_folder.addFolder('y-axis options');
        y_folder.add(config.y, 'min', -100, 100).step(1).onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        y_folder.add(config.y, 'max', -100, 100).step(1).onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        y_folder.add(config.y, 'step', 0, 100).step(1).onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        y_folder.addColor(config.y, 'color').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        var grid_lines_folder = grid_folder.addFolder('grid line options');
        grid_lines_folder.add(config.grid_lines, 'show').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        grid_lines_folder.addColor(config.grid_lines, 'color').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        grid_lines_folder.add(config.grid_lines, 'width', 0.5, 10).step(0.1).onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        var labels_folder = grid_folder.addFolder('label options');
        labels_folder.add(config.labels, 'show').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        labels_folder.addColor(config.labels, 'color').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        labels_folder.add(config.labels, 'font').onChange(function (value)
        {
            var feed_name = get_feed_name(grid_folder.domElement);
            var feed_obj = feeds[feed_name];
            feed_obj.draw_grid();
        });
        $(grid_folder.domElement).closest('li.folder').find('li.title').addClass('subfolder');
    }

    function is_host_contains_port(uri) { return new RegExp(":([0-9]+)", 'g').test(uri); }

    function add_new_feed(frontend_config, config, feed_path)
    {
        var grid_types = ['image', 'stream', 'track'];
//         var feed_name=feed_path.replace(/\[/g,"\\[").replace(/\]/g,"\\]").replace(/\//g,"\\/");
        var feed_name = feed_path_to_id(feed_path);
        if (!('type' in config)) { config.type = 'image'; }
        if (grid_types.indexOf(config.type) >= 0) { config.grid = new GridOptions(config.grid); }
        if (config.type == 'stream')
        {
            config.show = true;
            if (!('stream' in config)) { config.stream = {autoplay: false}; }
            if (!('url' in config))
            {
                var xpath = config.xpath || feed_path;
                if (typeof frontend_config.websocket == 'object')
                {
                    var host = frontend_config.websocket.host == undefined ? globals.host : frontend_config.websocket.host;
                    var port = frontend_config.websocket.port;
                    var endpoint = frontend_config.websocket.endpoint;
                    config.url = 'ws://' + host + ':' + port + '/' + endpoint;
                }
                else
                {
                    config.url = frontend_config.websocket;
                }
                config.url = config.url.replace("${HOST}", globals.host);
                config.url += '?xpath=' + xpath + '&data_uri=true';
            }
        }
        else
        {
            config.view = Feed.views[0];
            if (!('refresh' in config)) { config.refresh = {}; }
            if (!('auto' in config.refresh)) { config.refresh.auto = 'interval' in config.refresh && config.refresh.interval > 0; }
            if (!('interval' in config.refresh)) { config.refresh.interval = 2; }
            if (!('url' in config))
            {
                var host;
                if (globals.host[0] === '/')
                {
                    host = globals.host; // this is a relative path
                }
                else
                {
                    if (!globals.host.startsWith("http://")) { globals.host = "http://" + globals.host; }
                    host = is_host_contains_port(globals.host) ? globals.host : ( globals.host + ":" + globals.port );
                }
                // if (globals.host.indexOf(":") > 0) { host = globals.host; }
                if (!host.endsWith("/")) { host = host + "/"; }
                config.url = host + feed_path;
            }
        }
        if (config.type == 'text')
        {
            if (!('text' in config)) { config.text = {}; }
            if (!('show_items' in config.text)) { config.text.show_items = 1; }
        }
        else if (config.type == 'csv' || config.type == 'csv-table')
        {
            if (!('csv' in config)) { config.csv = {}; }
            if (!('show_items' in config.csv)) { config.csv.show_items = 1; }
            if (!('fields' in config.csv)) { config.csv.fields = ''; }
            if (!('min' in config.csv)) { config.csv.min = ''; }
            if (!('max' in config.csv)) { config.csv.max = ''; }
            if (!('min_color' in config.csv) || !config.csv.min_color) { config.csv.min_color = 'orange'; }
            if (!('max_color' in config.csv) || !config.csv.max_color) {config.csv.max_color = 'red'; }
            if (!('threshold_alert' in config.csv)) { config.csv.threshold_alert = false; }
        }
        else if (config.type == 'graph')
        {
            if (!('graph' in config)) { config.graph = {}; }
            if (!('min' in config.graph)) { config.graph.min = 0; }
            if (!('max' in config.graph)) { config.graph.max = 100; }
            if (!('thresholds' in config.graph)) { config.graph.thresholds = []; }
            config.graph.thresholds.sort(function (a, b) { return a.value - b.value; });
        }
        else if (config.type == 'track')
        {
            config.track = new TrackOptions(config.track);
        }
        else if (config.type == 'map')
        {
            config.map = new MapOptions(config.map, frontend_config);
        }
        if (!('alert' in config)) { config.alert = true; }
        var feed_obj = create_feed_fn(config.type, feed_name, feed_path, config);
        // if (config.type != 'form' && config.type != 'stream') {
        //     if (feed_obj.is_feed_inputs()) {
        //         feed_obj.input_container.empty();
        //         feed_obj.init();
        //         feed_obj.addListeners();
        //
        //         var form_ = $('<form>');
        //         feed_obj.load_inputs(form_);
        //         feed_obj.input_container.append(form_);
        //     }
        // }
        var folder = gui.addFolder(feed_path);
        folder.close();
        folder.add(feed_obj.config, 'url').onFinishChange(function (value)
        {
            var feed_name = get_feed_name(this.domElement);
            feeds[feed_name].reset();
        });
        if( config.type == 'stream' )
        {
            folder.add(feed_obj.config, 'show').onFinishChange(function (value)
            {
                var feed_name = get_feed_name(this.domElement);
                feeds[feed_name].toggle_show();
                if (!value) { gui.setProperty('auto', false, feed_name); }
            });
        }
        else
        {
            folder.add(feed_obj.config, 'view', Feed.views).onFinishChange(function (value)
            {
                var feed_name = get_feed_name(this.domElement);
                feeds[feed_name].update_view();
                if( value == 'hide' ) { gui.setProperty('auto', false, feed_name); }
            });
            folder.add(feed_obj.config.refresh, 'auto').name("auto refresh").onFinishChange(function (value)
            {
                var feed_name = get_feed_name(this.domElement);
                var feed_obj = feeds[feed_name];
                if( value && feed_obj.config.view == 'hide' ) { gui.setProperty('view', 'show', feed_name); }
                feed_obj.reset();
            });
            folder.add(feed_obj.config.refresh, 'interval', 0, 90).name("refresh interval").step(1).onFinishChange(function (value)
            {
                var feed_name = get_feed_name(this.domElement);
                feeds[feed_name].reset();
            });
            folder.add(feed_obj.config, 'alert').name('feed alert').onFinishChange(function (value)
            {
                var feed_name = get_feed_name(this.domElement);
                if (!value) { feeds[feed_name].alert(false); }
            });
            if (config.type == 'text')
            {
                folder.add(feed_obj.config.text, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function (value)
                {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].draw();
                });
            }
            if (config.type == 'csv' || config.type == 'csv-table')
            {
                folder.add(feed_obj.config.csv, 'show_items', 0, 20).name("show items").step(1).onFinishChange(function (value)
                {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].draw();
                });
                folder.add(feed_obj.config.csv, 'fields').onChange(function (value)
                {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].init_fields();
                });
                folder.add(feed_obj.config.csv, 'min').onFinishChange(function (value)
                {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].init_ranges();
                });
                folder.add(feed_obj.config.csv, 'max').onFinishChange(function (value)
                {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].init_ranges();
                });
                folder.add(feed_obj.config.csv, 'threshold_alert').name('threshold alert');
            }
            if (config.type == 'graph')
            {
                folder.add(feed_obj.config.graph, 'max').onFinishChange(function (value)
                {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].set_labels();
                });
                folder.add(feed_obj.config.graph, 'min').onFinishChange(function (value)
                {
                    var feed_name = get_feed_name(this.domElement);
                    feeds[feed_name].set_labels();
                });
            }
            if (config.type == 'track') { add_gui_track_options(folder, feed_obj.config.track); }
            if (config.type == 'map') { add_gui_map_options(folder, feed_obj.config.map); }
        }
        if (grid_types.indexOf(config.type) >= 0) { add_gui_grid_options(folder, feed_obj.config.grid); }
        feeds[feed_name] = feed_obj;
    }
    function load_feed_array(frontend_config, feeds, path)
    {
        for (var i = 0; i < feeds.length; i++)
        {
            base_controller.prototype.load_feed_items(frontend_config, feeds[i], path + "[" + i + "]");
        }
    }
    return base_controller;
});
