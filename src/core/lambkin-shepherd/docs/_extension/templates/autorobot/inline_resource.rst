.. module:: {{ fullname }}

    .. raw:: html

        <iframe id="{{ fullname }}.libdoc" src="{{ html_doc }}" style="height:200px;width:100%;border:none;overflow:hidden;"
                onload='window.parent.postMessage({type: "resize", height: document.documentElement.scrollHeight}, "*");'>
        </iframe>

        <script type="text/javascript">
            // listen for messages from the parent window
            window.addEventListener('message', function(event) {
              if (event.data.type === 'resize') {
                // resize the iframe to the height specified in the message
                iframe = document.getElementById("{{ fullname }}.libdoc");
                iframe.style.height = event.data.height + "px";
              }
            });
        </script>
