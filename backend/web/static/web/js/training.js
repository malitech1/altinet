const container = document.querySelector("[data-training-endpoint]");

if (container) {
  const endpoint = container.dataset.trainingEndpoint;
  const form = container.querySelector("[data-training-form]");
  const nameInput = form?.querySelector("[data-training-name]");
  const notesInput = form?.querySelector("[data-training-notes]");
  const startButton = form?.querySelector('[data-action="start-camera"]');
  const captureButton = form?.querySelector('[data-action="capture"]');
  const clearButton = form?.querySelector('[data-action="clear"]');
  const trainButton = form?.querySelector('[data-action="train"]');
  const statusElement = form?.querySelector('[data-training-status]');
  const videoElement = form?.querySelector('[data-training-video]');
  const placeholderElement = form?.querySelector('[data-training-placeholder]');
  const capturesContainer = form?.querySelector('[data-training-captures]');
  const emptyMessage = form?.querySelector('[data-training-empty]');
  const captureCanvas = form?.querySelector('[data-training-canvas]');

  /** @type {{ dataUrl: string; selected: boolean }} */
  const captures = [];
  let mediaStream = null;

  const getCsrfToken = () => {
    const name = "csrftoken";
    const cookies = document.cookie ? document.cookie.split(";") : [];
    for (const cookie of cookies) {
      const trimmed = cookie.trim();
      if (trimmed.startsWith(`${name}=`)) {
        return decodeURIComponent(trimmed.substring(name.length + 1));
      }
    }
    return null;
  };

  const setStatus = (message, isError = false) => {
    if (!statusElement) {
      return;
    }
    statusElement.textContent = message || "";
    statusElement.classList.toggle("text-danger", Boolean(isError));
    if (!isError) {
      statusElement.classList.add("text-muted");
    }
  };

  const stopCamera = () => {
    if (!mediaStream) {
      return;
    }
    mediaStream.getTracks().forEach((track) => {
      try {
        track.stop();
      } catch (error) {
        console.warn("Unable to stop media track", error);
      }
    });
    mediaStream = null;
    if (videoElement) {
      videoElement.srcObject = null;
      videoElement.setAttribute("hidden", "hidden");
    }
    if (placeholderElement) {
      placeholderElement.removeAttribute("hidden");
    }
    if (captureButton) {
      captureButton.disabled = true;
    }
  };

  const updateButtons = () => {
    const hasCaptures = captures.some((capture) => capture.selected);
    if (clearButton) {
      clearButton.disabled = captures.length === 0;
    }
    if (trainButton) {
      const nameFilled = Boolean(nameInput?.value.trim());
      trainButton.disabled = !hasCaptures || !nameFilled;
    }
  };

  const renderCaptures = () => {
    if (!capturesContainer) {
      return;
    }
    capturesContainer.innerHTML = "";

    if (emptyMessage) {
      emptyMessage.toggleAttribute("hidden", captures.length > 0);
    }

    captures.forEach((capture, index) => {
      const col = document.createElement("div");
      col.className = "col";
      col.innerHTML = `
        <div class="card h-100 shadow-sm ${capture.selected ? "border-primary" : "border-secondary"}">
          <img src="${capture.dataUrl}" class="card-img-top" alt="Captured frame ${index + 1}" />
          <div class="card-body p-2 d-flex flex-column gap-2">
            <div class="form-check">
              <input class="form-check-input" type="checkbox" id="capture-${index}" ${
                capture.selected ? "checked" : ""
              } />
              <label class="form-check-label small" for="capture-${index}">
                Use this photo
              </label>
            </div>
            <button type="button" class="btn btn-outline-danger btn-sm" data-action="remove">
              Remove
            </button>
          </div>
        </div>
      `;

      const checkbox = col.querySelector("input[type='checkbox']");
      const removeButton = col.querySelector("[data-action='remove']");

      if (checkbox) {
        checkbox.addEventListener("change", (event) => {
          capture.selected = event.target.checked;
          renderCaptures();
          updateButtons();
        });
      }

      if (removeButton) {
        removeButton.addEventListener("click", () => {
          captures.splice(index, 1);
          renderCaptures();
          updateButtons();
        });
      }

      capturesContainer.appendChild(col);
    });
  };

  const startCamera = async () => {
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      setStatus("Camera access is not supported in this browser.", true);
      return;
    }

    if (mediaStream) {
      return;
    }

    try {
      mediaStream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: "user" },
        audio: false,
      });
      if (videoElement) {
        videoElement.srcObject = mediaStream;
        videoElement.removeAttribute("hidden");
      }
      if (placeholderElement) {
        placeholderElement.setAttribute("hidden", "hidden");
      }
      if (captureButton) {
        captureButton.disabled = false;
      }
      setStatus("Camera ready. Capture a variety of angles.");
    } catch (error) {
      console.error("Unable to start camera", error);
      setStatus("We couldn't access the camera. Check browser permissions and try again.", true);
    }
  };

  const captureFrame = () => {
    if (!videoElement || !captureCanvas) {
      return;
    }

    const width = videoElement.videoWidth;
    const height = videoElement.videoHeight;
    if (!width || !height) {
      setStatus("Camera is warming up. Try capturing again in a moment.", true);
      return;
    }

    captureCanvas.width = width;
    captureCanvas.height = height;
    const context = captureCanvas.getContext("2d");
    if (!context) {
      setStatus("Unable to capture image from the camera stream.", true);
      return;
    }

    context.drawImage(videoElement, 0, 0, width, height);

    let dataUrl = "";
    try {
      dataUrl = captureCanvas.toDataURL("image/jpeg", 0.92);
    } catch (error) {
      console.error("Failed to encode capture", error);
      setStatus("We couldn't save that capture. Try again.", true);
      return;
    }

    captures.push({ dataUrl, selected: true });
    renderCaptures();
    updateButtons();
    setStatus("Capture added. Deselect any frames you don't want to use.");
  };

  const clearCaptures = () => {
    captures.splice(0, captures.length);
    renderCaptures();
    updateButtons();
  };

  const submitTraining = async (event) => {
    event.preventDefault();
    if (!form || !trainButton) {
      return;
    }

    const selectedCaptures = captures
      .filter((capture) => capture.selected)
      .map((capture) => capture.dataUrl);

    if (!nameInput || selectedCaptures.length === 0) {
      updateButtons();
      return;
    }

    const payload = {
      full_name: nameInput.value.trim(),
      notes: notesInput?.value.trim() || "",
      images: selectedCaptures,
    };

    if (!payload.full_name) {
      updateButtons();
      return;
    }

    const csrfToken = getCsrfToken();

    trainButton.disabled = true;
    trainButton.classList.add("disabled");
    setStatus("Training in progress… this may take a moment.");

    try {
      const response = await fetch(endpoint, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Accept: "application/json",
          ...(csrfToken ? { "X-CSRFToken": csrfToken } : {}),
        },
        body: JSON.stringify(payload),
      });

      const data = await response.json().catch(() => null);

      if (!response.ok || !data?.success) {
        const errorMessage = data?.error || "Training failed. Please try again.";
        throw new Error(errorMessage);
      }

      setStatus(data.message || "Training complete.");
      form.reset();
      clearCaptures();
      stopCamera();
    } catch (error) {
      console.error("Training request failed", error);
      setStatus(error.message || "Unable to train this profile.", true);
    } finally {
      trainButton.classList.remove("disabled");
      updateButtons();
    }
  };

  nameInput?.addEventListener("input", updateButtons);
  startButton?.addEventListener("click", startCamera);
  captureButton?.addEventListener("click", captureFrame);
  clearButton?.addEventListener("click", clearCaptures);
  form?.addEventListener("submit", submitTraining);

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      stopCamera();
    }
  });

  window.addEventListener("beforeunload", () => {
    stopCamera();
  });
}
