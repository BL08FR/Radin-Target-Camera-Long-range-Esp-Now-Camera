
setInterval(function getData()
{
    var xhttp = new XMLHttpRequest();

    xhttp.onreadystatechange = function()
    {
        if(this.readyState == 4 && this.status == 200)
        {
            document.getElementById("RSSIval").innerHTML = this.responseText;
        }
    };

    xhttp.open("GET", "readRSSI", true);
    xhttp.send();
}, 1000);