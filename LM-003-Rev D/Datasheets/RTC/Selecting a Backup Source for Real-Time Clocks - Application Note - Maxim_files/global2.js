var login={
            status:false,
	    data:{}
	};




$(document).ready(
    function() {
var url = window.location.toString();
        if(url.match(/maximintegrated.com\/en/) || url.match(/maximintegrated.com\/[^('jp'|'cn')]/)){document.getElementById('lang_en').style.display='none';document.getElementById('lang_en_mob').style.display='none';addthis_config =
              {
                "data_track_addressbar":false,
                services_compact: "twitter,facebook,google_plusone_share,linkedin,pinterest_share,gmail,email,print"
              };}
        else if(url.match(/china.maximintegrated.com/) || url.match(/maximintegrated.com\/cn/)){document.getElementById('lang_cn').style.display='none';document.getElementById('delimit').style.display='none';document.getElementById('lang_cn_mob').style.display='none';addthis_config =
              {
                "data_track_addressbar":false,
                services_exclude:"facebook,twitter,cleanprint,favorites",
                services_compact: "sinaweibo,qzone,gmail,email,print",
                      ui_language : "zh"
              };}
        else if(url.match(/maximintegrated.com\/jp/) || url.match(/japan.maximintegrated.com/)){document.getElementById('lang_jp').style.display='none';document.getElementById('lang_jp_mob').style.display='none';addthis_config =
             {
                "data_track_addressbar":false,
                services_exclude:"gmail,cleanprint,favorites",
                services_compact: "twitter,facebook,google_plusone_share,email,print",
                ui_language : "ja"
              };}

if(url.match(/print=enabled/)){
          window.focus();
          window.print();
        }


    /*Eloqua tracking click on tabs/pdfs*/
        if (typeof customUrl === 'undefined'){customUrl=url;}
        //var _pageUrl = customUrl || window.location.protocol + "//" + window.location.hostname + window.location.pathname;
        var _pageUrl = window.location.protocol + "//" + window.location.hostname + window.location.pathname;
	if (_pageUrl.substr(_pageUrl.length-1) === "/"){ _pageUrl = _pageUrl.substr(0, _pageUrl.length-1); }
        $("ul.nav-tabs li a").click(function(){
                if (($(this).attr("href").substr(0,1) === "#") && (typeof _elq !== "undefined") && (typeof _elq.trackEvent === "function")){
                        var _label = $(this).text();
                        if (typeof _label.trim === "function"){
                                _label = _label.trim();
                        }
                        _label = _label.replace(/\s/g,"_");
                        _elq.trackEvent( _pageUrl + "/" + _label );
                }
        });
        //tracking clicks on links to PDFs
        $("a[href$='.pdf'],a[href$='.PDF']").click(function(){
                if ((typeof _elq !== "undefined") && (typeof _elq.trackEvent === "function")){
                        _elq.trackOutboundLink(this);
                        return false;
                }
        });
    /*End of Eloqua tracking click on tabs/pdfs*/


    /*Fix by CKJ for data-toggle=tab*/ 
    	$("#myTab a").click(function () {
		var x = this.href;
		window.location.replace(x);
	});
    /*fix end*/
        $("#ib1").click(function() {
            $("#ib1info").slideToggle();
        });
        $("#ib2").click(function() {
            $("#ib2info").slideToggle();
        });
        $("#ib3").click(function() {
            $("#ib3info").slideToggle();
        });
        $("#parametricsearch").click(function() {
            $("#parametricsearchinfo").slideToggle();
        });
		$('.carousel').carousel({interval: 5000});
});

var Prev = '';
$(document).ready(function(){
	$('.accordion-toggle').click(function(){
		if(Prev == $(this).parent().next().attr('id')){
			$('#'+Prev).slideToggle('slow');
			Prev = '';
		}
		else{
		if(Prev != ''){
			$('#'+Prev).slideToggle('slow');
			Prev = $(this).parent().next().attr('id');
		}
		else{
			Prev = $(this).parent().next().attr('id');
		}
		$(this).parent().next().slideToggle('slow');
		}
	});
	$('.left-nav-section').hover(function() {
		$(this).parents('.accordion-body').attr('style','display:block');
		var p = $(this);
		var offset = p.offset();
		//$(this).find('.terlinks').attr('style', 'top:'+ p.offset().top + 'px; left:'+ (p.offset().left - 6) + 'px;');
		var body_padding = $('body').css('padding-top');
		var pad= parseInt(body_padding.replace('px',''));
		$(this).find('.terlinks').attr('style', 'top:'+ (p.offset().top - pad) + 'px; left:80%;');
		//$(this).find('.terlinks').attr('style', 'top:'+ p.offset().top + 'px; left:137px;');
		$(this).find('.terlinks').show().animate({
            'width': '263px'
        },260);
	  }, function() {
		$(this).find('.terlinks').css('width', '0');
		$(this).find('.terlinks').hide();
	  }
	);
});

$(function (){
 $("#productStatus").tooltip();
});

$(".modal-wide").on("show.bs.modal", function() {
  var height = $(window).height() - 200;
  $(this).find(".modal-body").css("max-height", height);
});   







/**
* Customized Login, My Bookmarks and My Cart Header widget javascript starts from here.
*/

/**
Login Widget Javascript starts here
*/
    
function setCookie(c_name, value, exdays) {

    var exdate = new Date();

    exdate.setDate(exdate.getDate() + exdays);

    var c_value = value + ((exdays == null) ? "" : "; expires=" + exdate.toUTCString())+ ";domain=.maximintegrated.com;path=/";

    document.cookie = c_name + "=" + c_value;

}



function addReturnUrlCookie()

{

    setCookie("MAX_GOAL",document.URL,365);

}

/**
Login Widget End Here
*/

/**
*My Bookmarks javascript starts here 
*/

function printExternal(url) {
var W = window.open(url);
setTimeout(function(){W.window.print();}, 5000);
}

/**
* My Bookmarks javascript ends here
*/

/**
* My Cart javascript starts here
*/
function mycart_widget()
{
        var cart_json = '{ "data" : [{"name" : "MAX_SHOP_SHOP", "url" : "https://shop.maximintegrated.com/storefront/shoppingcart.do?event=ShowShoppingCart&menuitem=ShoppingCart", "headers" : ["Maxim Part", "Quantity", "Price"],"divid" : "myCartOrder", "cook" : [0, 3, 8], "tableid" : "OrderTable", "emptymsg" : "No item(s) in <b>Buy</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabSample\') href=#myCartSample>Sample</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabQuote\') href=#myCartQuote>Quote</a></b> Carts?"}, {"name" : "MAX_SHOP_SAMPLE", "url" : "https://shop.maximintegrated.com/storefront/samplecart.do?event=ShowSampleCart&menuitem=SampleCart", "headers" : ["Maxim Part", "Quantity", "Customer Reference Number"], "divid" : "myCartSample", "cook" : [0, 2, 1], "tableid" : "SampleTable", "emptymsg" : "No item(s) in <b>Sample</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabOrder\') href=#myCartOrder>Buy</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabQuote\') href=#myCartSample>Quote</a></b> Carts?"}, {"name" : "MAX_SHOP_QUOTE","url" : "https://shop.maximintegrated.com/storefront/quotecart.do?event=showQuoteCart&menuitem=QuoteCart", "headers" : ["Maxim Part", "Quantity", "Competition Part", "Target Price (USD$)"], "divid" : "myCartQuote","cook" : [0, 2, 3,4], "tableid" : "QuoteTable", "emptymsg" : "No item(s) in <b>Quote</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabOrder\') href=#myCartOrder>Buy</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabSample\') href=#myCartSample>Sample</a></b> Carts?"}] }';
	var empty_msg = { MAX_SHOP_SHOP  : {
			 en : "No item(s) in <b>Buy</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabSample\') href=#myCartSample>Sample</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabQuote\') href=#myCartQuote>Quote</a></b> Carts?",
			 cn : '&#26242;&#26102;&#27809;&#26377;<b>&#35746;&#36141;</b>&#30340;&#20135;&#21697;&#12290;&#28857;&#20987;<b><a href="#myCartSample" onclick="tabchange(\'myTabSample\')" data-toggle="tab">&#26679;&#29255;</a></b>&#25110;&#32773;<b><a href="#myCartQuote" onclick="tabchange(\'myTabQuote\')" data-toggle="tab">&#35810;&#20215;</a></b>&#26597;&#35810;&#23545;&#24212;&#39033;&#30446;&#12290;',
			 jp : '<b>&#36092;&#20837;</b>&#12459;&#12540;&#12488;&#12395;&#12399;&#20309;&#12418;&#20837;&#12387;&#12390;&#12356;&#12414;&#12379;&#12435;&#12290;<b><a href="#myCartSample" onclick="tabchange(\'myTabSample\')" data-toggle="tab">&#12469;&#12531;&#12503;&#12523;</a></b>/<b><a href="#myCartQuote" onclick="tabchange(\'myTabQuote\')" data-toggle="tab">&#35211;&#31309;&#12418;&#12426;</a></b>&#12459;&#12540;&#12488;&#12434;&#12481;&#12455;&#12483;&#12463;&#12375;&#12390;&#12367;&#12384;&#12373;&#12356;&#12290;'
		    },
	MAX_SHOP_QUOTE : {
			 en : "No item(s) in <b>Quote</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabOrder\') href=#myCartOrder>Buy</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabSample\') href=#myCartSample>Sample</a></b> Carts?",
			 cn : '&#26242;&#26102;&#27809;&#26377;<b>&#35810;&#20215;</b>&#21333;&#12290;&#28857;&#20987;<b><a href="#myCartOrder" onclick="tabchange(\'myTabOrder\')" data-toggle="tab">&#35746;&#36141;</a></b>&#25110;&#32773;<b><a href="#myCartSample" onclick="tabchange(\'myTabSample\')" data-toggle="tab">&#26679;&#29255;</a></b>&#26631;&#31614;&#39029;&#26597;&#35810;&#23545;&#24212;&#39033;&#30446;&#12290;',
			 jp: '<b>&#35211;&#31309;&#12418;&#12426;</b>&#12459;&#12540;&#12488;&#12395;&#12399;&#20309;&#12418;&#20837;&#12387;&#12390;&#12356;&#12414;&#12379;&#12435;&#12290;<b><a href="#myCartOrder" onclick="tabchange(\'myTabOrder\')" data-toggle="tab">&#36092;&#20837;</a></b>/<b><a href="#myCartSample" onclick="tabchange(\'myTabSample\')" data-toggle="tab">&#12469;&#12531;&#12503;&#12523;</a></b>&#12459;&#12540;&#12488;&#12434;&#12481;&#12455;&#12483;&#12463;&#12375;&#12390;&#12367;&#12384;&#12373;&#12356;&#12290;'
			}, 
	MAX_SHOP_SAMPLE : {
			 en : "No item(s) in <b>Sample</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabOrder\') href=#myCartOrder>Buy</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabQuote\') href=#myCartQuote>Quote</a></b> Carts?",
			 cn : '&#30003;&#35831;&#26679;&#29255;&#35831;&#25552;&#20132;<a href="https://support.maximintegrated.com/cn/sales/samples/">&#26679;&#29255;&#30003;&#35831;&#21333;</a>&#12290;&#20102;&#35299;&#26679;&#29255;&#30003;&#35831;&#36827;&#24230;&#65292;&#35831;&#32852;&#31995;&#23458;&#25143;&#26381;&#21153;&#12290;',
		         jp : '<b>&#12469;&#12531;&#12503;&#12523;</b>&#12459;&#12540;&#12488;&#12395;&#12399;&#20309;&#12418;&#20837;&#12387;&#12390;&#12356;&#12414;&#12379;&#12435;&#12290;<b><a href="#myCartOrder" onclick="tabchange(\'myTabOrder\')" data-toggle="tab">&#36092;&#20837;</a></b>/<b><a href="#myCartQuote" onclick="tabchange(\'myTabQuote\')" data-toggle="tab">&#35211;&#31309;&#12418;&#12426;</a></b>&#12459;&#12540;&#12488;&#12434;&#12481;&#12455;&#12483;&#12463;&#12375;&#12390;&#12367;&#12384;&#12373;&#12356;&#12290;'
		      }, 
	go_to	     : {
			en : 'GO TO CART',
			cn : '&#21435;&#36141;&#29289;&#36710;',
			jp : '&#12459;&#12540;&#12488;&#12434;&#35211;&#12427;'
		      
		      }
		      };

        var cook = document.cookie;
	var lang = Lang.code;
        var cookieJson = JSON.parse(cart_json);
        for(var c =0; c < cookieJson.data.length; c++)
        {
                var cReg = new RegExp('(^|;)\\s*'+cookieJson.data[c].name+'=([^;]+)');
                var f=cook.match(cReg);
                var val = '';
                val=unescape(RegExp.$2);
		var url = '';
		url = cookieJson.data[c].url;
                if(! val.match(/:/g))
                {
                        document.getElementById(cookieJson.data[c].divid).innerHTML = '<div class="row"> <div class="col-sm-12" id=""><br />'+empty_msg[cookieJson.data[c].name][lang]+'</div></div>';
                }
                else
                {
                        val  = val.replace(/\~$/, '');
                        var cart_arr = val.split("~");
                        var head = '<div class="row"> <div class="col-sm-12" id=""><br /><table class="table table-bordered table-striped table-hover table-condensed"><thead><tr>';
                        for(var h =0; h < cookieJson.data[c].headers.length; h++)
                        {
                                var right_td = '<th>';
                                if(h == cookieJson.data[c].headers.length -1) { right_td = '<th class="table-right-head">'; }
                                head += right_td+cookieJson.data[c].headers[h]+'</th>';
                        }

                        var table_html = head+'</tr></thead>';
                        var table_rec = '<tbody>';
                        var total = 0;
                        for(var i=0; i<cart_arr.length; i++)
                        {
                                table_rec += '<tr>';
                                var cart = cart_arr[i].split(":");

                                for(var j=0;j< cookieJson.data[c].cook.length; j++)
                                {
                                        var ind = cookieJson.data[c].cook[j];
                                        var data = (cart[ind]) ? cart[ind] : 'N/A';
                                        if(cookieJson.data[c].name == 'MAX_SHOP_SHOP' && ind == 8) { if(data)data = (data) ? '$'+data : '$0.0'; }
                                        if(cookieJson.data[c].name == 'MAX_SHOP_QUOTE' && ind == 4) { if(data)data = (data) ? '$'+data : '$0.0'; }
                                        table_rec += '<td>'+data+'</td>';
                                        if(j == 2 && cookieJson.data[c].name == 'MAX_SHOP_SHOP') {
                                                total = parseFloat(total)+parseFloat(cart[cookieJson.data[c].cook[j]]);
                                        }
                                }
                                table_rec += '</tr>';
                        }
			url = cookieJson.data[c].url;
                        if(cookieJson.data[c].name == 'MAX_SHOP_SHOP') { table_rec += '<tr><td colspan=2>Subtotal</td><td>$'+(Math.round(total * 100) / 100).toFixed(2)+'</td>'; }
                        document.getElementById(cookieJson.data[c].divid).innerHTML = table_html+table_rec+'</tbody></table>';
                }
		document.getElementById(cookieJson.data[c].divid).innerHTML = document.getElementById(cookieJson.data[c].divid).innerHTML+'<div class="right-float"><button type="button" class="btn btn-danger btn-sm uppercase" onclick="javascript:gotocart(\''+url+'\');"">'+empty_msg['go_to'][lang]+'</button></div>';
        }
}

function tabchange(a_i)
{
        var deac = (a_i == 'myTabSample') ? ['myTabOrder', 'myTabQuote'] : (a_i == 'myTabOrder') ? ['myTabQuote', 'myTabSample'] : ['myTabSample', 'myTabOrder'];
        document.getElementById(a_i).className = 'uppercase active';
        document.getElementById(deac[1]).className = 'uppercase';
        document.getElementById(deac[0]).className = 'uppercase';
}

function gotocart(url)
{
	window.location = url;
}
	 
/**
* My Cart javascript ends here
*/

/**
* My Cart javascript starts here

function mycart_widget()
{
        var cart_json = '{ "data" : [{"name" : "MAX_SHOP_SHOP", "url" : "https://shop.maximintegrated.com/storefront/shoppingcart.do?event=ShowShoppingCart&menuitem=ShoppingCart", "headers" : ["Maxim Part", "Quantity", "Price"],"divid" : "myCartOrder", "cook" : [0, 3, 8], "tableid" : "OrderTable", "emptymsg" : "No item(s) in <b>Buy</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabSample\') href=#myCartSample>Sample</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabQuote\') href=#myCartQuote>Quote</a></b> Carts?"}, {"name" : "MAX_SHOP_SAMPLE", "url" : "https://shop.maximintegrated.com/storefront/samplecart.do?event=ShowSampleCart&menuitem=SampleCart", "headers" : ["Maxim Part", "Quantity", "Customer Reference Number"], "divid" : "myCartSample", "cook" : [0, 2, 1], "tableid" : "SampleTable", "emptymsg" : "No item(s) in <b>Sample</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabOrder\') href=#myCartOrder>Buy</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabQuote\') href=#myCartSample>Quote</a></b> Carts?"}, {"name" : "MAX_SHOP_QUOTE","url" : "https://shop.maximintegrated.com/storefront/quotecart.do?event=showQuoteCart&menuitem=QuoteCart", "headers" : ["Maxim Part", "Quantity", "Competition Part", "Target Price (USD$)"], "divid" : "myCartQuote","cook" : [0, 2, 3,4], "tableid" : "QuoteTable", "emptymsg" : "No item(s) in <b>Quote</b> Cart.Have you checked your <b><a data-toggle=tab onclick=tabchange(\'myTabOrder\') href=#myCartOrder>Buy</a></b> or <b><a data-toggle=tab onclick=tabchange(\'myTabSample\') href=#myCartSample>Sample</a></b> Carts?"}] }';

        var cook = document.cookie;
        var cookieJson = JSON.parse(cart_json);
        for(var c =0; c < cookieJson.data.length; c++)
        {
                var cReg = new RegExp('(^|;)\\s*'+cookieJson.data[c].name+'=([^;]+)');
                var f=cook.match(cReg);
                var val = '';
                val=unescape(RegExp.$2);
		var url = '';
		url = cookieJson.data[c].url;
                if(! val.match(/:/g))
                {

                        document.getElementById(cookieJson.data[c].divid).innerHTML = '<div class="row"> <div class="col-sm-12" id=""><br />'+cookieJson.data[c].emptymsg+'</div></div>';
                }
                else
                {
                        val  = val.replace(/\~$/, '');
                        var cart_arr = val.split("~");
                        var head = '<div class="row"> <div class="col-sm-12" id=""><br /><table class="table table-bordered table-striped table-hover table-condensed"><thead><tr>';
                        for(var h =0; h < cookieJson.data[c].headers.length; h++)
                        {
                                var right_td = '<th>';
                                if(h == cookieJson.data[c].headers.length -1) { right_td = '<th class="table-right-head">'; }
                                head += right_td+cookieJson.data[c].headers[h]+'</th>';
                        }

                        var table_html = head+'</tr></thead>';
                        var table_rec = '<tbody>';
                        var total = 0;
                        for(var i=0; i<cart_arr.length; i++)
                        {
                                table_rec += '<tr>';
                                var cart = cart_arr[i].split(":");

                                for(var j=0;j< cookieJson.data[c].cook.length; j++)
                                {
                                        var ind = cookieJson.data[c].cook[j];
                                        var data = (cart[ind]) ? cart[ind] : 'N/A';
                                        if(cookieJson.data[c].name == 'MAX_SHOP_SHOP' && ind == 8) { if(data)data = (data) ? '$'+data : '$0.0'; }
                                        if(cookieJson.data[c].name == 'MAX_SHOP_QUOTE' && ind == 4) { if(data)data = (data) ? '$'+data : '$0.0'; }
                                        table_rec += '<td>'+data+'</td>';
                                        if(j == 2 && cookieJson.data[c].name == 'MAX_SHOP_SHOP') {
                                                total = parseFloat(total)+parseFloat(cart[cookieJson.data[c].cook[j]]);
                                        }
                                }
                                table_rec += '</tr>';
                        }
			url = cookieJson.data[c].url;
                        if(cookieJson.data[c].name == 'MAX_SHOP_SHOP') { table_rec += '<tr><td colspan=2>Subtotal</td><td>$'+total+'</td>'; }
                        document.getElementById(cookieJson.data[c].divid).innerHTML = table_html+table_rec+'</tbody></table>';
                }
		document.getElementById(cookieJson.data[c].divid).innerHTML = document.getElementById(cookieJson.data[c].divid).innerHTML+'<div class="right-float"><button type="button" class="btn btn-danger btn-sm uppercase" onclick="javascript:gotocart(\''+url+'\');"">Go To Cart</button></div>';
        }
}

function tabchange(a_i)
{
        var deac = (a_i == 'myTabSample') ? ['myTabOrder', 'myTabQuote'] : (a_i == 'myTabOrder') ? ['myTabQuote', 'myTabSample'] : ['myTabSample', 'myTabOrder'];
        document.getElementById(a_i).className = 'uppercase active';
        document.getElementById(deac[1]).className = 'uppercase';
        document.getElementById(deac[0]).className = 'uppercase';
}

function gotocart(url)
{
	window.location = url;
}
	 
/**
* My Cart javascript ends here
*/
/**
* Cookie consent banner
*/
function checkCookieConsent() {
    var consent_cookie=getCookieConsent("CConsent");
    if (consent_cookie!=1) {
		document.getElementById('Cookieconsent').style.display='block';
		//document.getElementsByClassName('row')[0].style.marginTop= "10px";
		//$('.row').css('margin-top','10px');
		if(!document.getElementById('hdr')) { //para search page body 
		document.body.style.paddingTop = "86px"; }
    }else{
                document.getElementById('Cookieconsent').style.display='none';
		//document.getElementsByClassName('row')[0].style.marginTop= "0px"
		if(!document.getElementById('hdr')) { //para search page body
		document.body.style.paddingTop = "76px"; }
	}
}
function getCookieConsent(cname) {
    var name = cname + "=";
    var ca = document.cookie.split(';');
    for(var i=0; i<ca.length; i++) {
        var c = ca[i];
        while (c.charAt(0)==' ') c = c.substring(1);
        if (c.indexOf(name) != -1) return c.substring(name.length,c.length);
    }
    return "";
}

function setCookieConsent() {
var now = new Date();
var time = now.getTime();
time += 3600 * 1000 * 24 * 396
now.setTime(time);
document.cookie="CConsent=1;domain=.maximintegrated.com; expires="+now.toUTCString()+";path=/";
checkCookieConsent();
}
$(function() { checkCookieConsent(); });
/**
* Cookie consent banner ends here
*/
function getUrlLanguage()
{
    var lang="en";
    if(document.URL.indexOf("/cn/")>0 || document.URL.indexOf("/cn.html")>0)
        lang="zh";
    else if(document.URL.indexOf("/jp/")>0 || document.URL.indexOf("/jp.html")>0)
        lang="ja";
    return lang;
}
function getUrlCountry()
{
    var country="en";
    if(document.URL.indexOf("/cn/")>0 || document.URL.indexOf("/cn.html")>0)
        country="cn";
    else if(document.URL.indexOf("/jp/")>0 || document.URL.indexOf("/jp.html")>0)
        country="jp";
    return country;
}

function unescapeHtml(unsafe) {
    return unsafe
        .replace(/&amp;/g, "&")
        .replace(/&lt;/g, "<")
        .replace(/&gt;/g, ">")
        .replace(/&quot;/g, "\"")
        .replace(/&#039;/g, "'");
}
var personalInfoStore={};
var hostPrefix = "https://"+window.location.hostname;
function initBookmark() {
    var allBookmarks = personalInfoStore.bookmarks;
    $("#bookmarkContent").html('');
    if (allBookmarks != null && typeof allBookmarks !== 'undefined' && !(!allBookmarks || /^\s*$/.test(allBookmarks))) {
        var page_img = '<!--[if lte IE 8]><img src="https://www.maximintegrated.com/etc/designs/maximintegrated/_images/icons/page_icon.png" /><![endif]--><!--[if gt IE 8]><img src="https://www.maximintegrated.com/etc/designs/maximintegrated/_images/icons/page_icon.svg" /><![endif]--><!--[if !IE]> --><img src="https://www.maximintegrated.com/etc/designs/maximintegrated/_images/icons/page_icon.svg" /><!-- <![endif]-->';
        $("#bookmarkContent").html('');
        var values = allBookmarks.split("###");
        var max = (values.length > 7) ? 7 : values.length;
        var content = '';
        for (var i = 0; i < max; i++) {
            var parts = values[i].split("|");
			if(parts.length>2)
            	content += '<p class="clear">' + page_img + '<a href="' + parts[0] + '">' + unescapeHtml(parts[2]) + '</a></p>';
            else
                console.log("bookmark error:"+values+" i:"+i+" value"+values[i]);
        }
        $("#bookmarkContent").html(content);
    }
}
function initMyMaxim()
{
	$("#sortable-new").empty();
    var fullName = "Welcome, " + personalInfoStore.givenName + " " + personalInfoStore.familyName + "!";
    $("#fullNameUser").text(fullName);
    var mymaxsortOrder = personalInfoStore.mymaxOrder;
    var mymaxData = personalInfoStore.mymaxim;
    if (mymaxData != null && typeof mymaxData !== 'undefined' && !(!mymaxData || /^\s*$/.test(mymaxData))) {
        var mymaxArray = jQuery.parseJSON(decodeURIComponent(mymaxData));
        var isPublish=document.URL.indexOf("/content/maximintegrated")>=0?false:true;
        for (var i = 0; i < mymaxArray.length; i++) {
			var imageUrl = mymaxArray[i].mymaximage;
            var label = mymaxArray[i].mymaxlabel.replace(/\+/g, ' ');
            var link = mymaxArray[i].mymaxurl;
			if(isPublish)
            {
                var country = getUrlCountry();
                link = link.replace("/en/","/"+country+"/").replace("/cn/","/"+country+"/").replace("/jp/","/"+country+"/").replace("/en.html","/"+country+".html/").replace("/jp.html","/"+country+".html/").replace("/cn.html","/"+country+".html/");
            }
            else{
                var lang = getUrlLanguage();
                link = link.replace("/en/","/"+lang+"/").replace("/zh/","/"+lang+"/").replace("/ja/","/"+lang+"/").replace("/en.html","/"+lang+".html/").replace("/ja.html","/"+lang+".html/").replace("/zh.html","/"+lang+".html/");
            }
			if(link.indexOf("/")==0)
				link = hostPrefix+link;
            var newid = imageUrl.slice(-15);
            var newInput = $(document.createElement('li')).attr("class", 'mymax-img').attr("id",newid);
            newInput.after().html(
                    '<div><a href="' + link + '" class="mymax-link"><img class="mymax-image" src="' + imageUrl + '" /><span class="mymax-label">'+label+'</span></a></div>');
            newInput.appendTo("#sortable-new");
        }
        if(typeof mymaxsortOrder!="undefined")
        {	
	        $('.mymax-img')
	            .sort(
	                function(a, b) {
	                    return mymaxsortOrder.indexOf(a.id) > mymaxsortOrder
	                        .indexOf(b.id)
	                }).appendTo('#sortable-new');
        }
    }
}
var hasNewBulletin=false;
function initMyMaximBulletin()
{
    $('#newsortable-preview-new').empty();
    var count = 0;
    var bulletinDataStr = personalInfoStore.mymaximBulletin;
    if (bulletinDataStr != null && typeof bulletinDataStr !== 'undefined' && !(!bulletinDataStr || /^\s*$/.test(bulletinDataStr))) {
        var bulletinData = jQuery.parseJSON(decodeURIComponent(bulletinDataStr));
        for (var i = 0; i < bulletinData.length; i++) {
            var splitImgLink = bulletinData[i].bulletinType.split("-o.");
            var remainingUrl = splitImgLink[0];
            var hoverImage = remainingUrl + ".png";
            var newInput = null;
            if (bulletinData[i].isNew == "Yes") {
                newInput = $(document.createElement('li')).attr("class", 'max-bullet-new');
                count++;
            } else {
                newInput = $(document.createElement('li')).attr("class", 'max-bullet');
            }
			var link = bulletinData[i].bulletinUrl;
			if(link.indexOf("/")==0)
				link = "https://www.maximintegrated.com"+link;
            var newid = "imgid" + i;
            newInput.after().html('<div onmouseover="hover(\'' + newid + '\',\'' + hoverImage + '\');" onmouseout="unhover(\'' + newid + '\',\'' + bulletinData[i].bulletinType + '\');" ><a href="' + link + '" title="' + bulletinData[i].bulletinLabel + '" onclick="javascript:showLoadingIndicator(\'mybulletin-spinner\');"><img src="' + bulletinData[i].bulletinType + '" id="' + newid + '" class="icon"  />' + bulletinData[i].bulletinTitle.replace(/\+/g, ' ') + '</a></div> <br>');
            newInput.appendTo("#newsortable-preview-new");
        }
        
        if (count > 0) {
			hasNewBulletin=true;
			$('.nb-indicator').html('' + count);
            $('.nb-indicator').css('display', 'inline-block');
            $('.nb-indicator-modal').html('' + count);
            $('.nb-indicator-modal').css('display', 'block');
        } else {
            $('.nb-indicator').css('display', 'none');
            $('.nb-indicator-modal').css('display', 'none');
        }
    }
}
function resetMyMaximBulletin()
{
    var bulletinDataStr = personalInfoStore.mymaximBulletin;
    if (bulletinDataStr != null && typeof bulletinDataStr !== 'undefined' && !(!bulletinDataStr || /^\s*$/.test(bulletinDataStr))) {
        var bulletinData = jQuery.parseJSON(decodeURIComponent(bulletinDataStr));
        for (var i = 0; i < bulletinData.length; i++) {
            bulletinData[i].isNew = "No";
        }
        bulletinDataStr = encodeURIComponent(JSON.stringify(bulletinData));
        personalInfoStore.mymaximBulletin=bulletinDataStr;
		persistProfile(personalInfoStore);
    }
}
function add_bmark() {
    var title = $(document).find("title").text();
    addBookmarks(title, document.URL);
    //bookMarkProp(title, document.URL);
}

function addBookmarks(title, bookmark) {
    if(login.status)
    {
        var bookmarkstatus;
    
        var rightNow = new Date();
        var res = rightNow.toISOString().slice(0, 10);
        var newbookmark=bookmark + "|" + res + "|" + title;
        var updatedBookmarks;
    
        var bookmarkarray = new Array();
    
        var authorizableId = personalInfoStore.authorizableId;
        var currentbookmarks = personalInfoStore.bookmarks;
    
        if(!currentbookmarks || typeof currentbookmarks =="undefined" ||  /^\s*$/.test(currentbookmarks))
        {
            updatedBookmarks =newbookmark;
        }else if (currentbookmarks.indexOf("###") > -1) {
            bookmarkarray = currentbookmarks.split('###');
            for (var i = 0; i < bookmarkarray.length; i++) {
                var seperatedurl = bookmarkarray[i].split("|");
    
                if (seperatedurl[0].indexOf(bookmark) > -1) {
                    bookmarkstatus = "duplicate";
                    updatedBookmarks = currentbookmarks;
                } else {
                    updatedBookmarks = currentbookmarks + "###" + newbookmark;
                }
    
            }
        } else if (currentbookmarks.indexOf("|") > 0) {
            var newty = currentbookmarks.split("|");
    
            if (newty[0].indexOf(bookmark) > -1) {
                bookmarkstatus = "duplicate";
                updatedBookmarks = currentbookmarks;
            } else if (currentbookmarks.length > 1) {
                updatedBookmarks = currentbookmarks + "###" + newbookmark;
    
            } else {
                updatedBookmarks = newbookmark;
            }
        }
    
        if (bookmarkstatus != "duplicate") {
            var formData = "authorizableId=" + encodeURIComponent(authorizableId) + "&" + encodeURIComponent("profile/bookmarks") + "=" + encodeURIComponent(encodeURIComponent(updatedBookmarks));
            jQuery.ajax({
                url: hostPrefix+"/bin/AccountModificationServlet",
                type: "POST",
                contentType: "application/x-www-form-urlencoded",
                data: formData,
                xhrFields: {
                    withCredentials: true
                },
                success: function(data) {
                    if (data.status) {
                        personalInfoStore.bookmarks=updatedBookmarks;
						persistProfile(personalInfoStore);
                        initBookmark();
                        if('jp'==getUrlCountry())
                            alert("Bookmarkが追加されました。");
                        else
                            alert("Bookmark has been added successfully.");
                    } else
                        alert(data.error);
                },
                error: function(jqXHR, exception) {
                    if (jqXHR.status === 0) {
                        //alert("Not connect. Verify Network.");
                    } else if (jqXHR.status == 404) {
                        alert("Requested page not found. [404]");
                    } else if (jqXHR.status == 500) {
                        alert("Internal Server Error [500].");
                    } else if (exception === 'parsererror') {
                        alert("Requested JSON parse failed.");
                    } else if (exception === 'timeout') {
                        alert("Time out error.");
                    } else if (exception === 'abort') {
                        alert("Ajax request aborted.");
                    } else {
                        alert("Uncaught Error: " + jqXHR.responseText);
                    }
                }
            });
        } else {
            alert("Bookmark Already Added");
        }
    }
}
function checkTermsAccept(){
    var termsAccept = personalInfoStore.policyAcceptance;
    if (termsAccept == "false"){    
        if(document.URL.indexOf("/mymaxim/profile_preference.html")<0)
            window.location.replace("https://www.maximintegrated.com/"+getUrlCountry()+"/mymaxim/profile_preference.html");
    }   
}

function setMaximConfidentialLink() {
	var max_conf_url = personalInfoStore.max_conf_url;
	if($('#prodConfidentailLink').length && max_conf_url && max_conf_url != "") {
		$('#prodConfidentailLink').attr("href", max_conf_url+qvid).show();
        $('#prodConfidentailLink').click(function(){ 
             window.open(max_conf_url+qvid, 'maximConfidential', 'menubar=1,scrollbars=1,resizable=1,width=930,height=500'); 
             return false;
        });
    }
}
function sendSamlResponse(){
	jQuery.ajax({ 
        url:hostPrefix+"/bin/MaximSingleSignOnServlet", 
        type:"POST",
        dataType: "json",
        success:function(data){
            if(data.status)
            {
                var form = document.createElement("form");
                form.setAttribute("method", "post");
                form.setAttribute("action", data.postDst);
                form.setAttribute("target", "formresult");

                hiddenField = document.createElement("input");
                hiddenField.setAttribute("type", "hidden");
                hiddenField.setAttribute("name", "SAMLResponse");
                hiddenField.setAttribute("value", data.saml);
                form.appendChild(hiddenField);

                document.body.appendChild(form);
                window.open('support_request.html', 'formresult', 'scrollbars=yes,menubar=yes,height=600,width=1024,resizable=yes,status=yes');
                form.submit();
            }
            else
                alert(data.error);
        },
        complete:function( jqXHR, textStatus){
            $('#address-spinner').hide();
        },
        error: function(jqXHR, exception) {
            if (jqXHR.status === 0) {
                alert("Not connect. Verify Network.");
            } else if (jqXHR.status == 404) {
                alert("Requested page not found. [404]");
            } else if (jqXHR.status == 500) {
                alert("Internal Server Error [500].");
            } else if (exception === 'parsererror') {
                alert("Requested JSON parse failed.");
            } else if (exception === 'timeout') {
                alert("Time out error.");
            } else if (exception === 'abort') {
                alert("Ajax request aborted.");
            } else {
                alert("Uncaught Error: " + jqXHR.responseText);
            }
        }
    });
}
var navBarProfileCallback = function()
{
	if(personalInfoStore.isLoggedIn=="true" || personalInfoStore.isLoggedIn==true)
	{
		login.status=true;
		var country=getUrlCountry();
		if (country == 'en' || country == 'jp') {
			$('.myMaximSpan').html(personalInfoStore.givenName+" "+personalInfoStore.familyName).attr("style","text-transform:capitalize");
		}
		else {
			$('.myMaximSpan').html(personalInfoStore.familyName+" "+personalInfoStore.givenName).attr("style","text-transform:capitalize");
		}
		$('.myMaximLink').removeClass('disablelink');
		$('#myMaximLinkImg').removeClass('disablelink');
		$('#myMaximLogoutDiv').show();
		$('#myMaximLoginDiv').hide();
		$('#myMaximLogoutDl').removeAttr("style");
		$('#myMaximLoginDl').hide();
		$('#mymi-icon').attr("src",personalInfoStore.avatar);
		$('#mymi-icon-mobile').attr("src",personalInfoStore.avatar).show();
		initBookmark();
		setMaximConfidentialLink();
		initMyMaxim();
		initMyMaximBulletin();
		checkTermsAccept();
		if("true"==personalInfoStore.softlaunch)
        {
	        var requestTrackerName="My Support Portal";
	        var leftLinks = $('#leftmenu').find('a');
	        $.each( leftLinks, function(index, value) {
	        	if($(value).attr('href').indexOf('tech_support/router.mvp')>0)
	            {
	                $(value).attr('href','javascript:sendSamlResponse()');
					$(value).find('div').text(requestTrackerName);
	                return;
	            }
	        });
        }
	}
	else{
		$('.myMaximSpan').html("MyMaxim");
		$('.myMaximLink').addClass('disablelink');
		$('#myMaximLinkImg').addClass('disablelink');
		$('#myMaximLogoutDiv').hide();
		$('#myMaximLoginDiv').show();
		$('#myMaximLogoutDl').hide();
		$('#myMaximLoginDl').show();
		$('#mymi-icon').attr("src","https://www.maximintegrated.com/etc/designs/maximintegrated/_images/icons/mymaxim_icon.png");
		$('#mymi-icon-mobile').attr("src","https://www.maximintegrated.com/etc/designs/maximintegrated/_images/icons/mymaxim_icon.png").hide();
		$('#bookmarkNoLoginDiv').show();
		$('#bookmarkDiv').hide();
		if(typeof lpMTagConfig !== 'undefined')
		{
			lpMTagConfig.vars.push(["session","MemberCenter_id","0"]);
			lpMTagConfig.vars.push(["session","Priority",""]);
		}
	}
}
function getLocalCachedProfile()
{
	var data=null;
	if(store.enabled)
		data=store.get("maxim_profile");
	return data;
}
function persistProfile(personalInfo)
{
	if(store.enabled){
		store.set("maxim_profile", personalInfo);
		store.set("maxim_profile_legacy_changed", "true");
	}
}
function resetCQCacheFlag()
{
	if(store.enabled){
		store.set("maxim_profile_cq_changed", "false");
	}
}
function isCQCahceChanged()
{
	var changed="false";
	if(store.enabled){
		changed = store.get("maxim_profile_cq_changed");
	}
	return "true"==changed;
}
function hasProfileData(personalInfo)
{
	return personalInfo!=null && typeof personalInfo !="undefined" && personalInfo.authorizableId != null 
	&& typeof personalInfo.authorizableId != "undefined" && personalInfo.authorizableId !="" && personalInfo.authorizableId !="anonymous";
}
function loadProfile()
{
	if(typeof personalInfoLink != "undefined" && typeof personalInfoLink.url != "undefined"){
		jQuery.ajax({
			url: personalInfoLink.url.replace("www.maximintegrated.com",window.location.hostname)+"?t="+(new Date()).getTime(),
			type: "GET",
			dataType: "json",
			xhrFields: {
				withCredentials: true
			},
			success: function(data) {
				personalInfoStore=data;
				if(hasProfileData(personalInfoStore))
				{
					personalInfoStore.lastRefreshed = new Date().getTime();
					if(store.enabled){
						store.set("maxim_profile", personalInfoStore);
						resetCQCacheFlag();
					}
				}
				navBarProfileCallback();
			},
			error: function(jqXHR, exception) {
				if (jqXHR.status === 0) {
					//alert("Not connect. Verify Network.");
				} else if (jqXHR.status == 404) {
					alert("Requested page not found. [404]");
				} else if (jqXHR.status == 500) {
					alert("Internal Server Error [500].");
				} else if (exception === 'parsererror') {
					alert("Requested JSON parse failed.");
				} else if (exception === 'timeout') {
					alert("Time out error.");
				} else if (exception === 'abort') {
					alert("Ajax request aborted.");
				} else {
					alert("Uncaught Error: " + jqXHR.responseText);
				}
			}
		});
	}
}
function isProfileOld(personalInfo)
{
	return personalInfo==null || typeof personalInfo =="undefined" || personalInfo.lastRefreshed == null 
	|| typeof personalInfo.lastRefreshed == "undefined" || personalInfo.lastRefreshed +3600000 < new Date().getTime();
}
function isDifferentLanguage(personalInfo)
{
	return personalInfo==null || typeof personalInfo =="undefined" || personalInfo.language == null 
	|| typeof personalInfo.language == "undefined" || getUrlLanguage()!= personalInfo.language;
}
$(function(){
	personalInfoStore = getLocalCachedProfile();
	if(!hasProfileData(personalInfoStore)){
		loadProfile();
	}else if(isProfileOld(personalInfoStore)){
		loadProfile();
	}else if(isDifferentLanguage(personalInfoStore)){
		loadProfile();
	}else if(isCQCahceChanged()){
		loadProfile();
	}else
		navBarProfileCallback();

	$("#bookmarksLink").click(function() {
		if (login.status) {
			$('#bookmarkNoLoginDiv').hide();
			$('#bookmarkDiv').show();
			initBookmark();
		}
		else
		{
			$('#bookmarkNoLoginDiv').show();
			$('#bookmarkDiv').hide();
		}

		$('#bookmarksModal').modal('show');
	});
	$('[data-toggle="tooltip"]').tooltip();

	refreshProfile();		
});
function refreshProfile()
{
	setTimeout(function (){
		loadProfile();
		refreshProfile();
	}, 900000);
}
function lastClickedDate() {
    var authorizableId = personalInfoStore.authorizableId;
    var bulletindate = new Date();
    var mymaximdate = bulletindate.toISOString();
    var lang=getUrlLanguage();
    var formData = "authorizableId=" + encodeURIComponent(authorizableId) + "&" + encodeURIComponent("profile/lastClickedDate") + "=" + encodeURIComponent(mymaximdate)+ "&" + encodeURIComponent("profile/language") + "=" + lang;
    resetMyMaximBulletin();
    jQuery.ajax({
        url: hostPrefix+"/bin/AccountModificationServlet",
        type: "POST",
        contentType: "application/x-www-form-urlencoded",
        data: formData,
        xhrFields: {
            withCredentials: true
        },
        success: function(data) {
            initMyMaximBulletin();
        },
        error: function(jqXHR, exception) {
            if (jqXHR.status === 0) {
                //alert("Not connect. Verify Network.");
            } else if (jqXHR.status == 404) {
                alert("Requested page not found. [404]");
            } else if (jqXHR.status == 500) {
                alert("Internal Server Error [500].");
            } else if (exception === 'parsererror') {
                alert("Requested JSON parse failed.");
            } else if (exception === 'timeout') {
                alert("Time out error.");
            } else if (exception === 'abort') {
                alert("Ajax request aborted.");
            } else {
                alert("Uncaught Error: " + jqXHR.responseText);
            }
        }
    });
}

function updateMyMaxList(IDs, action) {
    var authorizableId = personalInfoStore.authorizableId;
    var mymaxlist = IDs;
    var mymaxstring = mymaxlist.join(",");
    var mymaxData = "authorizableId=" + encodeURIComponent(authorizableId) + "&" + encodeURIComponent("profile/mymaxOrder") + "=" + encodeURIComponent(mymaxstring);
	personalInfoStore.mymaxOrder=mymaxstring;
	persistProfile(personalInfoStore);
    if(action>0)
    {
        var bulletindate = new Date();
        var lang=getUrlLanguage();
    	var mymaximdate = bulletindate.toISOString();
		mymaxData+="&" + encodeURIComponent("profile/lastClickedDate") + "=" + encodeURIComponent(mymaximdate)+ "&" + encodeURIComponent("profile/language") + "=" + lang;
        resetMyMaximBulletin();
        initMyMaximBulletin();
    }
    jQuery.ajax({
        url: hostPrefix+"/bin/AccountModificationServlet",
        type: "POST",
        contentType: "application/x-www-form-urlencoded",
        data: mymaxData,
        xhrFields: {
            withCredentials: true
        },
        success: function(data) {
            if (data.status) {
                ;
            } else
                alert(data.error);
        },
        error: function(jqXHR, exception) {
            if (jqXHR.status === 0) {
                //alert("Not connect. Verify Network.");
            } else if (jqXHR.status == 404) {
                alert("Requested page not found. [404]");
            } else if (jqXHR.status == 500) {
                alert("Internal Server Error [500].");
            } else if (exception === 'parsererror') {
                alert("Requested JSON parse failed.");
            } else if (exception === 'timeout') {
                alert("Time out error.");
            } else if (exception === 'abort') {
                alert("Ajax request aborted.");
            } else {
                alert("Uncaught Error: " + jqXHR.responseText);
            }
        }
    });
}

$(function() {
    var IDs = [];
    var newFormat = [];
    var action = 0;
    var counterValue = parseInt($('.bubble').html());

    $('#bulletinTab').click(function() {
        action++;
        $('#bulletinTab').css("border-bottom","5px solid #6e2585");
        $('#mymaximTab').css("border-bottom","none");
    });
    $('#mymaximTab').click(function() {
		if(hasNewBulletin)
    	{
			lastClickedDate();
			hasNewBulletin=false;
		}
        $('#bulletinTab').css("border-bottom","none");
        $('#mymaximTab').css("border-bottom","5px solid #6e2585");
    });
    $("#sortable-new").sortable();

    $("#sortable-new").disableSelection();

    $('#myMaxDialog').on('hidden.bs.modal', function() {
        IDs = [];
        $('.mymax-image').each(function() {
            var currentimgid = this.src.slice(-15);
            IDs.push(currentimgid);

        });
        var mymaxstring = IDs.join(",");
        var mymaxsortOrder = personalInfoStore.mymaxOrder;
        if(mymaxstring!=mymaxsortOrder)
        	updateMyMaxList(IDs, action);
    });

    //sortMyMaxim();

    function removeAnimation() {
        setTimeout(function() {
            $('.bubble').removeClass('animating')
        }, 1000);
    }
	$('input[name=resource]').val(document.URL); 
});

function hover(id, imgsrc) {
    document.getElementById(id).src = imgsrc;
}

function unhover(id, imgsrc) {
    document.getElementById(id).src = imgsrc;
}
function showLoadingIndicator(imgId)
{
	$('#'+imgId).show();
    return false;
}