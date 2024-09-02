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

define('CsvFeed', ["jquery", 'TextFeed'], function ($, Feed, TextFeed) {
    //var $ = require('jquery');
    //require("jquery_ui");
    var Feed = require('Feed');
    var TextFeed = require("TextFeed");

    var CsvFeed = function (feed_name, feed_path, config) {
        this.base = Feed;
        this.base(feed_name, feed_path, config);
        this.init();
    };
    CsvFeed.prototype = Object.create(TextFeed.prototype);
    CsvFeed.prototype.init = function ()
    {
        this.init_fields();
        this.init_ranges();
    };
    CsvFeed.prototype.init_fields = function ()
    {
        this.target.find('thead').empty();
        if (this.config.csv.fields) 
        {
            var tr = $('<tr>');
            if( this.config.type == 'csv-table' )
            {
                var fields = this.config.csv.fields.split(',');
                console.log( "===> Fields are: " + fields);
                for( var i in fields ) { $('<th class="text-center">').text( fields[i] ).appendTo( tr ); }
            }
            else
            {
                $('<th>').text(this.config.csv.fields).appendTo(tr);
            }
            this.target.find('thead').append(tr);
        }
    };
    CsvFeed.prototype.init_ranges = function ()
    {
        this.min = {};
        var min = this.config.csv.min.split(',');
        for (var i in min)
        {
            if (min[i] && !isNaN(min[i])) { this.min[i] = Number(min[i]); }
        }
        this.max = {};
        var max = this.config.csv.max.split(',');
        for (var i in max)
        {
            if (max[i] && !isNaN(max[i])) { this.max[i] = Number(max[i]); }
        }
    };
    CsvFeed.prototype.onload_ = function (data)
    {
        var data_color;
        var column_has_space;
        var tbody = $('<tbody>');
        var fields = this.config.csv.fields.split(',');
        if (data)
        {
            data = data.split('\n').map(function (value, index) { return value.split(','); });
            var out_of_range = false;
            data_color = new Array(data.length);
            var columns = 0;
            for (var i in data)
            {
                if (columns < data[i].length) { columns = data[i].length; }
            }
            var column_has_space = new Array(columns);
            for (var i in data)
            {
                for (var j = 0; j < data[i].length; ++j)
                {
                    if (data[i][j].match(/ /)) { column_has_space[j] = true; }
                }
            }
            var column_is_number = new Array(columns);
            for (var i in data)
            {
                for (var j = 0; j < data[i].length; ++j)
                {
                    if (data[i][j].match(/^[+-]?\d+(\.\d+)?$/)) { column_is_number[j] = true; }
                }
            }
            for (var i in data)
            {
                data_color[i] = new Array(data[i].length);
                for (var j in data[i]) {
                    var value = data[i][j];
                    if (value && !isNaN(value))
                    {
                        if (j in this.min && Number(value) < this.min[j])
                        {
                            out_of_range = true;
                            data_color[i][j] = this.config.csv.min_color;
                        }
                        else if (j in this.max && Number(value) > this.max[j])
                        {
                            out_of_range = true;
                            data_color[i][j] = this.config.csv.max_color;
                        }
                        if (this.config.type == 'csv' && data_color[i][j])
                        {
                            data[i][j] = '<span style="color:' + data_color[i][j] + '">' + data[i][j] + '</span>';
                        }
                    }
                }
            }
            this.alert(this.config.csv.threshold_alert && out_of_range);
        }
        else
        {
            data = '&nbsp;'
        }
        if( this.config.type == 'csv-table' )
        {
            if( typeof data === 'object' )
            {
                for( var i in data )
                {
                    if (data[i].length == 1 && !data[i][0]) { continue; }
                    var tr = $('<tr>');
                    for (var j in data[i])
                    {
                        var td = $('<td>');
                        if ((!column_has_space[j]) && column_is_number[j]) { td.addClass('text-right'); }
                        var value = data[i][j]
                        if( column_is_number[j] && globals.isMobile ) { value = parseFloat( value ).toFixed(3); }
                        if( data_color[i][j] ) // todo: quick and dirty, give up for now; do it properly!
                        {
                            var pre = $('<pre>');
                            pre.css('color', data_color[i][j]);
                            value = pre.text( value );
                        }
                        else if( !this.config.csv.html )
                        {
                            var pre = $('<pre>');
                            value = pre.text( value );
                        }
                        else
                        {
                            //value = '<div style="text-align:right">' + value + '</div>'
                        }

                        // Storing the current row and column index in the cell's data attribute
                        // Store using jQuery's .data() method:
                        td.data('rowIndex', i); // Store the row Index
                        td.data('colIndex', j); // Store the column Index
                        td.data('rowData', data[i]); // Store the entire row data

                        tr.append( td.append( value ) );

                        // Click handler
                        td.on('click', function () {
                            // Retrieve stored data from the clicked cell
                            var rowIndex = String($(this).data('rowIndex'));
                            var colIndex = $(this).data('colIndex');
                            var rowData = $(this).data('rowData');

                            // Retrieve the corresponding field name
                            var fieldName = fields[colIndex];

                            // Output to the console
                            console.log("Clicked Cell Info:");
                            console.log("Row ID: ", rowIndex);
                            console.log("Column ID: ", fieldName);
                            console.log("Full Row Data: ", {
                                id: rowData[0], // as 'id' is always the 1st field
                                a: rowData[1], // 'a' field
                                b: rowData[2] // 'b' field
                            });

                            // Optional alert for debugging
                            
                            alert("You clicked on cell: " + $(this).text() + "\nRow ID: " + rowIndex + "\nField Name: " + fieldName + "\nFull Row Data: " + JSON.stringify({
                                id: rowData[0],
                                a: rowData[1],
                                b: rowData[2]
                            }));
                            

                            // alert("You clicked on cell: " + $(this).text());
                            // Additional click handling logic can go here
                            // console.log("User clicked on cell: " + $(this).text() + " whose Row ID is " + data[i] + " and whose Field name is " + data[j]);
                            // console.log("User clicked on cell: " + $(this).text() + " whose Row number is " + (i-1) + " with Row ID data value being " + $(data[i-1][0]).text() + 
                            //         " and whose Column number is " + parseInt(j) + " with data in cell being " + $(j+1).text());
                            // (i-1) because the script treats the column headers as a row# 1 which we wanna eliminate
                            // console.log("User clicked on cell: " + $(this).text() + " whose Row number is " + data[5][1]);
                        });
                    }
                    tbody.append(tr);
                }
            }
            else
            {
                tbody.append('<tr><td colspan="' + this.target.find('thead th').length + '">' + ( this.config.csv.html ? data : ( '<a href="www.google.com"><pre>' + data + '</pre>' ) ) + '</td></tr>');
            }
        }
        else
        {
            if( typeof data === 'object' )
            {
                data = data.map(function (value, index) { return value.join(','); }).join('\n');
            }
            if( data && data.length && data[data.length - 1] == '\n' )
            {
                data = data.substring( 0, data.length - 1 );
            }
            //tbody.append( '<tr><td>' + this.config.csv.html ? data : ( '<pre>' + data + '</pre>' ) + '</td></tr>' );
            tbody.append( '<tr><td>' + ( this.config.csv.html ? data : ( '<pre>' + data + '</pre>' ) ) + '</td></tr>' );
        }
        this.target.append(tbody);
        this.draw();
    };
    CsvFeed.prototype.draw = function ()
    {
        while( this.target.find('tbody').length > this.config.csv.show_items )
        {
            this.target.find('tbody').first().remove();
        }
    };
    return CsvFeed;
});



/*
Q1:
What is the difference between network monitoring and system monitoring in Software Development and DevOps? 
What are some examples of network monitoring open-source tools such as Nagios, Zabbix, etc?
List the purpose of each tool, some characteristics of the tools, their advantages & disadvantages, 
and in which situation to use each of the tool (when to use each of the tool)?

Summmary: Nagios: Comprehensive tool for both network and system monitoring, ideal for organizations with custom monitoring needs.
Zabbix: Enterprise-grade tool for scalable, centralized monitoring with advanced analysis and reporting.
Icinga: Modern, scalable solution suitable for distributed monitoring setups, especially in large environments.
Cacti: Best for visualizing network performance through graphing, focused on SNMP polling.
Netdata: Lightweight, real-time monitoring tool with excellent visualizations, suitable for smaller setups or supplementary monitoring.

Q2:
Briefly, what are some use cases of Zabbix, Nagios, Cacti, Icinga, Netdata, and Observium in terms of network monitoring tool?
*/