---
title: Engineering Portfolio
subtitle: Projects completed / in progress from undergrad and beyond
---

{: .box-success}
This is a demo post to show you how to write blog posts with markdown.  I strongly encourage you to [take 5 minutes to learn how to write in markdown](https://markdowntutorial.com/) - it'll teach you how to transform regular text into bold/italics/tables/etc.<br/>I also encourage you to look at the [code that created this post](https://raw.githubusercontent.com/daattali/beautiful-jekyll/master/_posts/2020-02-28-sample-markdown.md) to learn some more advanced tips about using markdown in Beautiful Jekyll.

**Here is some bold text**

## Here is a secondary heading

[This is a link to a different site](https://deanattali.com/) and [this is a link to a section inside this page](#local-urls).

Here's a table:

| Number | Next number | Previous number |
| :------ |:--- | :--- |
| Five | Six | Four |
| Ten | Eleven | Nine |
| Seven | Eight | Six |
| Two | Three | One |

Center image:
![Crepe](https://beautifuljekyll.com/assets/img/crepe.jpg){: .mx-auto.d-block :}

Image test: 6 (centering)

| Number | Next number | Previous number |
| :------ |:--- | :--- |
| ![ACRP](/assets/png/acrp_link.png){: .mx-auto.d-block :} | ![AstaZero](/assets/png/astazero_link.png){: .mx-auto.d-block :} | ![NRSL](/assets/png/nrsl_link.png){: .mx-auto.d-block :} |
| ![THON](/assets/png/thon_bot_link2.png){: .mx-auto.d-block :} | [![TonyPi](/assets/png/tonypi_link.png)](https://www.wikipedia.org/){: .mx-auto.d-block :} |


Here's a code chunk:

~~~
var foo = function(x) {
  return(x + 5);
}
foo(3)
~~~

And here is the same code with syntax highlighting:

```javascript
var foo = function(x) {
  return(x + 5);
}
foo(3)
```
[![TonyPi](https://github.com/willkraus9/willkraus9.github.io/blob/master/assets/png/tonypi_link.png)](https://www.wikipedia.org/)


And here is the same code yet again but with line numbers:

{% highlight javascript linenos %}
var foo = function(x) {
  return(x + 5);
}
foo(3)
{% endhighlight %}

## Boxes
You can add notification, warning and error boxes like this:

### Notification

{: .box-note}
**Note:** This is a notification box.

### Warning

{: .box-warning}
**Warning:** This is a warning box.

### Error

{: .box-error}
**Error:** This is an error box.

