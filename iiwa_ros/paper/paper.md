---
title: 'FRI-based ros_control driver for KUKA iiwa'
tags:
  - ROS
  - Compliant Control
  - iiwa
  - KUKA
authors:
  - name: Konstantinos Chatzilygeroudis
    orcid: 0000-0000-0000-0000
    equal-contrib: false
    affiliation: "1" # (Multiple affiliations must be quoted)
  - name: Author Without ORCID
    equal-contrib: false # (This is how you can denote equal contributions between multiple authors)
    affiliation: 2
  - name: Author with no affiliation
    corresponding: false # (This is how to denote the corresponding author)
    affiliation: 3
  - name: Matthias Mayr
    orcid: 0000-0000-0000-0000
    equal-contrib: false
    affiliation: "2, 3" # (Multiple affiliations must be quoted)
affiliations:
 - name: TBD
   index: 1
 - name: Faculty of Engineering (LTH), Lund University, Sweden
   index: 2
 - name: Wallenberg AI, Autonomous Systems and Software Program (WASP), Sweden
   index: 3
date: 29 November 2022
bibliography: paper.bib

---

# Summary
Todo:
- High-level introduction
- Feature list

# Statement of need
This library is used in these scientific contributions: @mayr22skireil, @mayr22priors, etc.

# Control Implementation
This contains some LaTeX example code. Feel free to remove.

Single dollars ($) are required for inline mathematics e.g. $f(x) = e^{\pi/x}$

Double dollars make self-standing equations:

$$\Theta(x) = \left\{\begin{array}{l}
0\textrm{ if } x < 0\cr
1\textrm{ else}
\end{array}\right.$$

You can also use plain \LaTeX for equations
\begin{equation}\label{eq:fourier}
\hat f(\omega) = \int_{-\infty}^{\infty} f(x) e^{i\omega x} dx
\end{equation}
and refer to \autoref{eq:fourier} from text.

# Citations

Citations to entries in paper.bib should be in
[rMarkdown](http://rmarkdown.rstudio.com/authoring_bibliographies_and_citations.html)
format.

For a quick reference, the following citation commands can be used:
- `@author:2001`  ->  "Author et al. (2001)"
- `[@author:2001]` -> "(Author et al., 2001)"
- `[@author1:2001; @author2:2001]` -> "(Author1 et al., 2001; Author2 et al., 2002)"


<!--
# Figures
Figures can be included like this:
![Caption for example figure.\label{fig:example}](flowchart.png)
and referenced from text using \autoref{fig:example}.

Figure sizes can be customized by adding an optional second parameter:
![Caption for example figure.](flowchart.png){ width=10% }
-->
# Acknowledgements


# References