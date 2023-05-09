.. rubric:: Resources

.. list-table::
{% for name, documentation in resources %}
   * - :obj:`{{name}}`
     - {{documentation}}
{%- endfor %}

.. toctree::
   :hidden:
{% for name, _ in resources %}
   {{name}}
{%- endfor %}
