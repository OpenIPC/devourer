# The swif-codec Repository

--------

**an Open-source sliding window FEC codec (IETF hackathon)**

Main goal is to develop an open-source C-language codec for a sliding window FEC code. These codes can boost performance of content delivery protocols in harsh environments where packet losses can be frequent, while keeping the FEC-related added latency low.

This development is done in the context of the "Coding for Efficient Network Communications" IRTF Research Group (NWCRG, [https://datatracker.ietf.org/rg/nwcrg]) and IETF hackathon [https://trac.ietf.org/trac/ietf/meeting/wiki/103hackathon].

This work has strong relationships with the Generic API I-D [https://datatracker.ietf.org/doc/draft-roca-nwcrg-generic-fec-api/] and RLC codes [https://datatracker.ietf.org/doc/draft-ietf-tsvwg-rlc-fec-scheme/] as examples of sliding window codes. Possible applications to QUIC [https://datatracker.ietf.org/doc/draft-swett-nwcrg-coding-for-quic/] and [https://datatracker.ietf.org/doc/draft-roca-nwcrg-rlc-fec-scheme-for-quic/], among others, are expected. 

--------

## Assumptions and limits of the SWIF-codec

The codec is not thread-safe: a single codec instance is limited to a single execution thread.
However a given application may use several threads, each of them creating their own codec instances.


## Reference documents

### API for the SWiF Codec

[Datatracker API Internet-draft](https://datatracker.ietf.org/doc/draft-roca-nwcrg-generic-fec-api/)

[GitHub current API Internet-draft](https://github.com/vincent-grenoble/generic_fec_api) - to be prefered to have the latest under-progress version

[API Slides (ietf102 nwcrg meeting)](https://github.com/irtf-nwcrg/rg-materials/blob/master/ietf102-2018-07_montreal/03_roca_generic_fec_api.pdf)

This dradft and associated slides discuss two things:
- what mechanisms should be in the SWiF Codec versus in the caller?
The codec is not supposed to include everything since parts of a FEC Scheme are clearly protocol specific (e.g., signalling aspects).

- a description of the codec API.

### RLC Sliding Window FEC Scheme whose parts can be reused

The [RLC FEC Scheme](https://datatracker.ietf.org/doc/draft-ietf-tsvwg-rlc-fec-scheme/) describes a Sliding Window FEC Scheme (i.e., the codec plus signalling aspects).
Parts of this document can be re-used (e.g., its PRNG and coding coefficients generation function, some of the internal parameters, etc.).

However the goal **IS NOT** to implement RLC. For instance in-network re-encoding should be supported by SWiF Codec that is totally out-of-scope for RLC.

The [ietf98 TSVWG slides 9-10-11](https://datatracker.ietf.org/meeting/98/materials/slides-98-tsvwg-sessb-63-fecframe-drafts-00) explain how Sliding Window FEC Codes, like RLC, work.


## Eight simple ideas for newcomers to start understanding FEC and Network Coding

### Idea 1-
#### "We focus on networks where a packet either arrives or is lost"
- we're not at PHY-layer, we are above in the protocol stack and potential bit errors have either been fixed or the packet dropped

### Idea 2-
#### "Encoding consists in adding redundancy (i.e., "repair packets") to the flow"

#### "Decoding consists in using redundancy (i.e., using "repair packets") to recover from packet losses"

### Idea 3-
#### "Math is not an obstacle to understand FEC and NC"
- it's essentially a matter of **linear combination and linear system resolution** (e.g., via basic Gaussian elimination)
- details (e.g., computations in a certain Finite Field) can be complex, but mastering them is not required

### Idea 4-
#### "There are roughly two categories of FEC codes: block codes and sliding window codes"
- block: segment the packet flow into blocks and apply FEC encoding per block, independently
- sliding window: an encoding window slides progressively over the packet flow, the encoder computes a linear combination of packets in this encoding window

### Idea 5-
#### "With large independant data objects, block codes are great, with streams of real-time data that have time correlation, sliding window codes are preferable"
- ... because splitting the application flow into blocks delays the moment when "repair packets" can be generated!
- code performance also depends on the type of losses (independant, uniform losses versus correlated, bursty losses) 

### Idea 6-
#### "Some codes are restricted to a single encoder (e.g., sender) and single decoder (e.g., receiver)"
- usually called FEC

#### "Other codes can be used within intermediate nodes (i.e., multiple encoders)"
- usually called Network Coding (NC)

### Idea 7-
#### "With NC, network equipments can perform FEC encoding to improve network usage"
- trivial example where a network equipment could reduce traffic (it sends a single "P1 XOR P2" packet (using multicast or broadcast) instead of sending both P1 and P2):
     
<pre><code>Alice        Wireless router          Bob    
  |    --P1-->      |                 |    
  |                 |     <--P2--     |    
  | <--P1 XOR P2--- | ---P1 XOR P2--> | (multicast or broadcast)
  |                 |                 |
recover P2                     Recover P1
</code></pre>

### Idea 8-
#### "One can use FEC and NC in a congestion friendly manner"
- only stupid persons will further overload a congested network with even more redundant traffic in the hope it may help!


## Open-source projects to get inspiration from

- Gardinet (Cedric Adjih): [https://gitlab.inria.fr/GardiNet/liblc/]
	Note that this implementation (1) does not provide clear separation between the codec and the protocol parts, and (2) has many compilation time defined constants.

- OpenFEC (Vincent Roca): [http://openfec.org]
	Note that this implementation does not include in its public version support for Sliding Window Codes.
	However it includes GF(256) support from Luigi Rizzo in the Reed-Solomon codec as well as an advanced performance evaluation framework (eperftool).

- NB: Kodo (Morten V. Pedersen, Steinwurf) is highly relevent as it provides several Sliding Window codecs [http://docs.steinwurf.com/kodo.html].
	However it is **NOT RECOMMENDED** to get inspiration from this project because of incompatible licences with the SWiFCodec project.


## Contact
vincent.roca@inria.fr
