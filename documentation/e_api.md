---
layout: page
title: API Documentation
category: Documentation
---

## Online

{:.table}
| Package | Documentation |
|---------|---------------|
{% for package in site.data.packages %}|{{ package.name }}|[master](https://projects.laas.fr/gepetto/doc/stack-of-tasks/{{ package.doc }}/master/doxygen-html/)|
{% endfor %}

### Finding the documentation on your system

If you have the stack of tasks installed on your system, the documentation corresponding to *your* version should also
be available in `${prefix}/share/doc/`
