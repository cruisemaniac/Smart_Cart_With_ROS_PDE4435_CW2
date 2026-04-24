# 🚀 LaTeX Report Setup Guide (ROS Smart Cart Project)

Welcome! This guide explains how to set up, build, and edit the **Smart Cart ROS Project Report** using VS Code and LaTeX locally.

## 🧠 Project Structure
```text
Reports/
└── Smart_Cart_CW2_PDE4435_Report_v1_0/
    ├── main.tex            # Main entry point
    ├── references.bib      # Bibliography file
    ├── images/             # Store all screenshots/diagrams here
    └── sections/           # Individual chapter .tex files
```

---

## 💻 1. Install VS Code
1. Download and install [VS Code](https://code.visualstudio.com/).
2. Open VS Code.

## 🧩 2. Install LaTeX Extension
Install the **LaTeX Workshop** extension:
1. Open VS Code.
2. Go to Extensions (`Ctrl + Shift + X`).
3. Search for: `LaTeX Workshop`.
4. Click **Install**.

## ⚙️ 3. Install LaTeX Engine (Required)
You must install a LaTeX distribution on your OS for the extension to work:

*   **Option A: TeX Live (Recommended for all OS)**  
    [Download TeX Live](https://www.tug.org/texlive/) (Full, robust installation).
*   **Option B: MiKTeX (Windows Preferred)**  
    [Download MiKTeX](https://miktex.org/) (Lightweight, auto-installs missing packages).

## 🧪 4. Verify Installation
Open your terminal and type:
```bash
latexmk -v
```
*   **If a version number shows:** You are ready! ✅
*   **If not:** Restart your PC or check your System PATH environment variables.

---
* if install MiKTeX and get an err as 'MiKTeX could not find the script engine 'perl'';
Install Perl (recommended)
---
1. Steps:
- Download from: https://strawberryperl.com/
- Install with default settings
- Restart your PC (important)

## 📄 5. Open Project
1. Open VS Code.
2. Go to `File` → `Open Folder`.
3. Select the `Smart_Cart_CW2_PDE4435_Report_v1_0` folder.

## 🚀 6. Build & View PDF
1. Open `main.tex`.
2. **Build PDF:** Press `Ctrl + Alt + B`.
3. **Live Preview:** Press `Ctrl + Alt + V`.

> **Tip:** You can see the code on the left and the PDF on the right simultaneously.

## ⚡ 7. Auto-Build on Save
To make the PDF update every time you hit `Ctrl + S`:
1. Press `Ctrl + ,` to open Settings.
2. Search for `latex-workshop.latex.autoBuild.run`.
3. Set it to `onSave`.

---

## 📁 8. Editing Rules
*   ✅ **Do Edit:** Files inside `sections/`, `images/`, and `references.bib`.
*   ❌ **Do Not Touch:** Auxiliary files (`.aux`, `.log`, `.fls`). These are generated automatically.
*   ⚠️ **Main File:** Only modify `main.tex` if you are adding new chapters or changing the document structure.

## 🖼️ 9. Adding Images
1. Place your image file in the `images/` folder.
2. Use this snippet in your `.tex` file:
```latex
\begin{figure}[ht]
    \centering
    \includegraphics[width=0.8\textwidth]{images/your_image_name}
    \caption{Description of the image}
    \label{fig:my_label}
\end{figure}
```

## 🧠 10. Common Issues
*   **`latexmk` not found:** Ensure your LaTeX engine (Step 3) is installed and you have restarted VS Code.
*   **PDF not updating:** Ensure `main.tex` is the active tab when you build, or right-click `main.tex` and select **Set as Workshop reference document**.